#!/usr/bin/env python
import math
import rclpy
import numpy as np
from collections import deque

RESEAT_BYPASS_MIN_FRAMES = 2      # post-commit frames the baseline must rebase onto before the
                                  # bypass window may close: a wall-clock-only close can expire
                                  # with ZERO admitted frames when the stream is degraded,
                                  # re-arming the pre-re-seat baseline loop
RESEAT_BYPASS_CEILING_S = 3.0     # hard cap so a dead/stale stream cannot pin the bypass open
import message_filters
from rclpy.node import Node
from std_msgs.msg import Bool, UInt8, UInt32
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, \
                      QoSReliabilityPolicy, \
                      QoSHistoryPolicy, \
                      QoSDurabilityPolicy
from rclpy.time import Time
from tf2_ros import TransformBroadcaster, TransformListener, Buffer
from px4_msgs.msg import EstimatorStatusFlags
from geometry_msgs.msg import TransformStamped, PoseStamped, Vector3Stamped, Quaternion, Pose
from rclpy.executors import MultiThreadedExecutor
from scipy.spatial.transform import Rotation as R
from isaac_ros_visual_slam_interfaces.srv import SetSlamPose
from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude, VehicleOdometry
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus


class vslam_reactor(Node):
    def __init__(self): 
        super().__init__('vslam_reactor_node')

        # Tunable parameters: declared with defaults, overridden by config/px4_vslam_reactor.yaml
        # (loaded by px4_vslam/launch/vslam.launch.py). Must run before sync_cache_sz is used below.
        self._load_params()

        # REACTOR CONTROL PARAMETERS
        self.init_flag = True
        self._last_px4_rx = self.get_clock().now()
        self._fmu_stamp_excursions = 0                      # FMU stamp skew clamp events (telemetry)
        self.vslam_status = 0
        self.vslam_busy = False
        self._vslam_busy_since = self.get_clock().now()
        self._seat_seq = 0
        self._seat_is_init = False   # kind of the in-flight seat: only ORIGIN seats bump the epoch   # re-seat generation: straggler SetSlamPose responses must not touch the successor's state
        self.new_set_pose_call = False
        self.ev_fusion_started = False                      # PX4 EV fusion
        self._vio_reset_epoch = 0                           # bumped on committed ORIGIN seats only -> EKF2 reset_counter
        self.last_set_pose_time = self.get_clock().now()

        # Post-re-seat jump-gate bypass: odom_velocity_gate only refreshes last_vslam_odom_msg on
        # the NOT-jump path, so the first frame after a re-seat is measured against the pre-re-seat
        # baseline, reads as a jump, re-seats again, and self-sustains. Stamped on the SUCCESS
        # response, not at dispatch: vslam_busy already drops every frame between dispatch and
        # response, so a dispatch-anchored window could expire before the first post-re-seat frame
        # is ever admitted. None = no window open (never re-seated, or the last window CLOSED).
        self._last_reseat_commit = None
        self._reseat_bypass_frames = 0   # frames stamped after the commit that rebased the baseline

        # Settle owner: only an ORIGIN settle may escalate to an origin re-injection on settle
        # timeout. A jump re-seat that fails to align must NEVER become a set_slam_pose(True)
        # mid-flight - that zeroes position AND yaw.
        self._settle_owner = 'origin'

        # Jump-re-seat burst limiter: committed JUMP re-seat timestamps inside the rolling window.
        # Origin re-injects are settle-timeout bounded and are NOT counted - only the unlimited
        # jump path is evidence that cuVSLAM is unrecoverable by re-seating.
        self._reseat_burst = deque()
        self._vo_healthy = True
        self._burst_unhealthy = False
        self._silence_unhealthy = False
        self._last_vslam_rx = None    # node-clock receipt of the last ingress-clean VO frame
        self._last_ev_pub = None      # node-clock time of the last EV output to vio_transform

        self.fmu_local_position = Vector3Stamped()
        self.last_vslam_odom_msg = Odometry()



        self.R_FRD_TO_FLU = R.from_euler('x', np.pi)

        ### TF BUFFERING ###########################################################################
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        ### CALLBACK GROUPS ########################################################################
        self.vslam_cbg = MutuallyExclusiveCallbackGroup()
        self.gen_processing_cbg = MutuallyExclusiveCallbackGroup()

        ### QoS PARAMETERS #########################################################################
        self.qos_fmu = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)
    
        self.qos_vslam = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)
        
        self.qos_transient = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1)

        ### PUBLISHERS #############################################################################
        self.pub_filtered_odom_ = self.create_publisher(Odometry,
                                                        '/visual_slam/filt_slam_odometry',
                                                         qos_profile=self.qos_vslam,
                                                         callback_group=self.vslam_cbg)
        
        self.pub_drone_odom_ = self.create_publisher(Odometry,
                                                     '/reactor/drone_odom', 
                                                     qos_profile=self.qos_vslam,
                                                     callback_group=self.vslam_cbg)

        self.pub_drone_pose_ = self.create_publisher(PoseStamped,
                                                     '/reactor/drone_pose',
                                                     qos_profile=self.qos_vslam,
                                                     callback_group=self.vslam_cbg)

        # EKF2 reset-epoch: incremented on every committed VSLAM re-seat and forwarded by
        # vio_transform into VehicleOdometry.reset_counter. Transient-local so a late/restarted
        # vio_transform latches the current value.
        self.pub_vio_reset_epoch_ = self.create_publisher(UInt8,
                                                          '/reactor/vio_reset_epoch',
                                                          qos_profile=self.qos_transient,
                                                          callback_group=self.vslam_cbg)
        self.pub_vio_reset_epoch_.publish(UInt8(data=self._vio_reset_epoch))

        # VO-health latch: FALSE once jump re-seats exhaust reseat_burst_max inside
        # reseat_burst_window_s, i.e. cuVSLAM is not recoverable BY re-seating and continuing would
        # only feed EKF2 a reset storm. Nothing on this platform consumes it: operator-facing
        # telemetry only, the reactor never commands a flight action. Latched, baseline True.
        self.pub_vo_healthy_ = self.create_publisher(Bool,
                                                     '/reactor/vo_healthy',
                                                     qos_profile=self.qos_transient,
                                                     callback_group=self.vslam_cbg)
        self.pub_vo_healthy_.publish(Bool(data=True))

        # 1 Hz on vslam_cbg: during an output blackout that group is idle, which is exactly
        # when this must still fire.
        self._ev_silence_timer = self.create_timer(1.0, self._ev_silence_check,
                                                   callback_group=self.vslam_cbg)


        ### SUBSCRIBERS ############################################################################
        self.vslam_status_sub = self.create_subscription(VisualSlamStatus,
                                                         '/visual_slam/status',
                                                         self.visual_slam_status_callback,
                                                         qos_profile=self.qos_vslam,
                                                         callback_group=self.gen_processing_cbg)
        
        self.est_status_sub = self.create_subscription(EstimatorStatusFlags,
                                                       '/fmu/out/estimator_status_flags',
                                                       self.est_status_callback,
                                                       self.qos_fmu,
                                                       callback_group=self.gen_processing_cbg)

        self._px4_odom_sub = self.create_subscription(VehicleOdometry,
                                                      '/fmu/out/vehicle_odometry',
                                                      self.px4_odom_callback,
                                                      qos_profile=self.qos_fmu,
                                                      callback_group=self.gen_processing_cbg)

        self._drone_odom_sub = message_filters.Subscriber(self, Odometry,
                                                          '/reactor/drone_odom',
                                                          qos_profile=self.qos_vslam,
                                                          callback_group=self.gen_processing_cbg)

        self._slam_odom_sub = message_filters.Subscriber(self, Odometry,
                                                         '/visual_slam/vis/slam_odometry',
                                                         qos_profile=self.qos_vslam,
                                                         callback_group=self.vslam_cbg)


        ### CACHES #################################################################################
        self._drone_odom_cache = message_filters.Cache(self._drone_odom_sub,
                                                       cache_size=self.sync_cache_sz)

        ### SERVICES ###############################################################################
        # The re-seat producer AND the SetSlamPose response live in vslam_cbg (MutuallyExclusive),
        # the same group as slam_odom_callback -> _reseat_on_jump. That makes every touch of
        # vslam_busy / _seat_seq / _last_reseat_commit / last_vslam_odom_msg single-threaded: with
        # the server in gen_processing_cbg, a service trigger and a jump re-seat could BOTH pass
        # vslam_check() and dispatch concurrent SetSlamPose calls, the loser undoing the winner.
        # NEVER block on this client's future from inside vslam_cbg (call_async only): the response
        # is processed by this same group, so a synchronous wait would deadlock it.
        self.set_slam_pose_client = self.create_client(SetSlamPose, 'visual_slam/set_slam_pose',
                                                       callback_group=self.vslam_cbg)

        self.trigger_slam_pose_service = self.create_service(Trigger, 'visual_slam/set_reactor_pose',
                                                             self.set_slam_pose_callback,
                                                             callback_group=self.vslam_cbg)

        ### TRANSFORM BROADCASTERS #################################################################            
        self.px4_tf_broadcaster = TransformBroadcaster(self)

        ### CALLBACK TIMERS ########################################################################
        # Bounded wait with a heartbeat log: a bare loop hung node construction without
        # logging (or hot-spun a core) when cuVSLAM never came up.
        while not self.set_slam_pose_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('waiting for visual_slam/set_slam_pose service...')

        ### CALLBACK REGISTRATIONS #################################################################
        self._slam_odom_sub.registerCallback(self.slam_odom_callback)


    def _load_params(self):
        # Tunable parameters; config/px4_vslam_reactor.yaml overrides these defaults (loaded by
        # px4_vslam/launch/vslam.launch.py). Angular gates are entered in degrees and converted.
        defaults = [
            ('vslam_stabilization_time', 1.0),   # in-node default matches yaml: it is the bypass-window floor, must not halve if the yaml fails to load
            ('lin_vel_gate', 5.0),
            ('ang_vel_gate_dps', 200.0),
            ('VO_rate_lim', 0.5),
            ('VO_pos_delta_lim', 0.4),
            ('sync_cache_sz', 300),
            ('align_yaw_deg', 2.0),
            ('align_pos_m', 0.10),
            ('set_pose_max_odom_age', 0.030),
            ('set_origin_settle_time', 10.0),
            ('fmu_stamp_max_skew_s', 0.5),
            ('set_pose_busy_timeout_s', 3.0),
            ('reseat_burst_max', 5),
            ('reseat_burst_window_s', 10.0),
            ('ev_silence_max_s', 2.0),
        ]
        self.declare_parameters('', defaults)
        self._param_names = [n for n, _ in defaults]
        self._apply_params()

    def _apply_params(self):
        for name in self._param_names:
            setattr(self, name, self.get_parameter(name).value)
        # Derived (degrees -> radians)
        self.ang_vel_gate = np.radians(self.ang_vel_gate_dps)
        self.align_yaw = np.radians(self.align_yaw_deg)

    def px4_odom_callback(self, msg):
        if not self.pose_ingress_ok(
                (float(msg.position[0]), float(msg.position[1]), float(msg.position[2])),
                (float(msg.q[1]), float(msg.q[2]), float(msg.q[3]), float(msg.q[0]))):
            # Dropped before the freshness stamp: garbage FMU frames (boot transients) must not
            # count as fresh odom for set_slam_pose, and a zero-norm q would kill R.from_quat.
            self.get_logger().error("FMU ingress: non-finite/zero-norm pose dropped",
                                    throttle_duration_sec=1.0)
            return
        now = self.get_clock().now()
        self._last_px4_rx = now

        # FMU stamp skew clamp: uXRCE timesync excursions can pass boot-relative or future
        # stamps straight through, corrupting tf2 buffers for the px4 frame.
        # Must be skew-vs-now, NOT a monotonicity guard: a +10s future stamp is still
        # monotonic. Duration.nanoseconds is signed, so future stamps yield negative skew.
        # clock_type must match now's (ROS_TIME); the Time() default is SYSTEM_TIME and
        # cross-clock subtraction raises TypeError.
        fmu_time = rclpy.time.Time(nanoseconds=msg.timestamp * 1000,
                                   clock_type=now.clock_type)
        skew_s = (now - fmu_time).nanoseconds * 1e-9
        if abs(skew_s) > self.fmu_stamp_max_skew_s:
            self._fmu_stamp_excursions += 1
            self.get_logger().warn(
                f"FMU stamp excursion: raw={msg.timestamp}us skew={skew_s:.3f}s "
                f"count={self._fmu_stamp_excursions} - re-stamping outputs with node clock",
                throttle_duration_sec=5.0)
            out_stamp = now.to_msg()
        else:
            # Normal path keeps the FMU stamp untouched: ordering fidelity matters to
            # downstream ApproximateTimeSynchronizer consumers.
            out_stamp = fmu_time.to_msg()

        # Position Conversion
        FMU_pos_frd = [float(msg.position[0]), 
                       float(msg.position[1]), 
                       float(msg.position[2])]
        FMU_pos_flu = self.position_frd_to_flu(FMU_pos_frd)

        # Quaternion Conversions
        FMU_q_frd = [float(msg.q[1]),
                     float(msg.q[2]),
                     float(msg.q[3]),
                     float(msg.q[0])]    # [x, y, z, w] in FRD
        FMU_q_flu = self.quat_frd_to_flu(FMU_q_frd)

        # map to px4 transform creation
        map_px4_t = TransformStamped()
        map_px4_t.header.stamp = out_stamp
        map_px4_t.header.frame_id = 'map'
        map_px4_t.child_frame_id = "px4" 
        map_px4_t.transform.translation.x = FMU_pos_flu[0]
        map_px4_t.transform.translation.y = FMU_pos_flu[1]
        map_px4_t.transform.translation.z = FMU_pos_flu[2]
        map_px4_t.transform.rotation.x = FMU_q_flu[0]
        map_px4_t.transform.rotation.y = FMU_q_flu[1]
        map_px4_t.transform.rotation.z = FMU_q_flu[2]
        map_px4_t.transform.rotation.w = FMU_q_flu[3]

        # ros-frame odometry message for drone
        drone_odom_msg = Odometry()
        drone_odom_msg.header.stamp = out_stamp
        drone_odom_msg.header.frame_id = "map"
        drone_odom_msg.child_frame_id = 'px4'
        drone_odom_msg.pose.pose.position.x = FMU_pos_flu[0]
        drone_odom_msg.pose.pose.position.y = FMU_pos_flu[1]
        drone_odom_msg.pose.pose.position.z = FMU_pos_flu[2]
        drone_odom_msg.pose.pose.orientation.x = FMU_q_flu[0]
        drone_odom_msg.pose.pose.orientation.y = FMU_q_flu[1]
        drone_odom_msg.pose.pose.orientation.z = FMU_q_flu[2]
        drone_odom_msg.pose.pose.orientation.w = FMU_q_flu[3]
        
        # drone posestamped message for viz
        drone_pose_msg = PoseStamped()
        drone_pose_msg.header = drone_odom_msg.header
        drone_pose_msg.pose = drone_odom_msg.pose.pose
       
        # broadcast for TF tree
        self.px4_tf_broadcaster.sendTransform(map_px4_t)

        # publish drone odom
        self.pub_drone_odom_.publish(drone_odom_msg)

        # publish drone posestamped
        self.pub_drone_pose_.publish(drone_pose_msg)

    # Potentially switch to message_filters.ApproximateTimeSynchronizer to trigger main odom callback
    def sync_msg(self, target_time: rclpy.time.Time, target_msg_cache: message_filters.Cache):
        try:
            msg = target_msg_cache.getElemBeforeTime(target_time)
        except IndexError:
            try:
                msg = target_msg_cache.getElemAfterTime(target_time)
            except IndexError:
                msg = None
        
        if msg is not None:
            msg_time = rclpy.time.Time.from_msg(msg.header.stamp)
            time_diff = abs((target_time - msg_time).nanoseconds)

            if time_diff > 100_000_000:
                return None
        
        return msg
    

    def service_response_callback(self, future, seq):
        # vslam_busy must clear on EVERY path (success, refusal, exception) or the reactor
        # stalls with EV publishing suppressed indefinitely. The epoch bump lives here, on success
        # only, so reset_counter can never precede the actual re-seat and a failed re-seat
        # never burns an epoch.
        if seq != self._seat_seq:
            # Straggler from a superseded re-seat (watchdog force-cleared it and a successor is
            # already in flight): touching vslam_busy or the epoch here corrupts the successor.
            self.get_logger().warn(
                f"SetSlamPose straggler response ignored (seq {seq} != {self._seat_seq})")
            return
        try:
            response = future.result()
            if response.success:
                if self._seat_is_init:
                    # ORIGIN seats only. A jump re-seat writes the FMU's OWN pose into
                    # cuVSLAM, so post-seat EV already agrees with EKF2 and there is
                    # nothing for a reset flag to force - bumping here commanded a full
                    # EKF2 vertical reset in flight for a value EKF2 already held. Any
                    # residual step is ordinary innovation; worst case EV de-latches and
                    # the re-latch performs the reset through the fusion-start path.
                    self._vio_reset_epoch = (self._vio_reset_epoch + 1) & 0xFF
                    self.pub_vio_reset_epoch_.publish(UInt8(data=self._vio_reset_epoch))
                # Opens the jump-gate bypass window here, on SUCCESS only: a refused/failed re-seat
                # moved nothing, so the pre-re-seat baseline is still the truth and the gate must
                # stay live against it.
                self._last_reseat_commit = self.get_clock().now()
                self._reseat_bypass_frames = 0
            else:
                self.get_logger().error(f"SetSlamPose failure: {response.message}")
        except Exception as e:
            self.get_logger().error(f"SetSlamPose call failed: {str(e)}")
        finally:
            self.vslam_busy = False


    def calculate_3d_displacement(self, pose_1: Pose, pose_2: Pose) -> float:
        np_pose_1 = np.array([pose_1.position.x,
                              pose_1.position.y,
                              pose_1.position.z])
        np_pose_2 = np.array([pose_2.position.x,
                              pose_2.position.y,
                              pose_2.position.z])
        displacement = np.linalg.norm(np_pose_2 - np_pose_1)
        return displacement

    def min_quat_theta(self, q1_orientation: Quaternion, q2_orientation: Quaternion) -> float:
        q1 = np.array([q1_orientation.x, q1_orientation.y, q1_orientation.z, q1_orientation.w])
        q2 = np.array([q2_orientation.x, q2_orientation.y, q2_orientation.z, q2_orientation.w])
        q1 /= np.linalg.norm(q1)
        q2 /= np.linalg.norm(q2) 
        dot_product = np.clip(np.abs(np.dot(q1, q2)), -1.0, 1.0)
        return 2 * np.arccos(dot_product)

    def yaw_delta(self, q1_orientation: Quaternion, q2_orientation: Quaternion) -> float:
        # Absolute yaw-only difference (radians), wrapped to [0, pi]. Used by the settle-exit
        # alignment check so yaw can be gated independently of roll/pitch.
        # UPPERCASE 'ZYX' = INTRINSIC z-y'-x'' (aerospace), element 0 = world yaw. Lowercase 'zyx'
        # is scipy's EXTRINSIC sequence, whose element 0 is a body-side angle that folds tilt into
        # the "yaw" difference (up to 0.69 deg at 20 deg tilt against the 2.0 deg settle budget).
        y1 = R.from_quat([q1_orientation.x, q1_orientation.y, q1_orientation.z, q1_orientation.w]).as_euler('ZYX')[0]
        y2 = R.from_quat([q2_orientation.x, q2_orientation.y, q2_orientation.z, q2_orientation.w]).as_euler('ZYX')[0]
        d = y1 - y2
        return abs(np.arctan2(np.sin(d), np.cos(d)))

    def position_frd_to_flu(self, pos_frd):
        return np.array([pos_frd[0],
                        -pos_frd[1],
                        -pos_frd[2]])

    def quat_frd_to_flu(self, q_frd):
        R_body_world_frd = R.from_quat(q_frd)
        R_body_world_flu = self.R_FRD_TO_FLU * R_body_world_frd * self.R_FRD_TO_FLU.inv()
        q_flu = R_body_world_flu.as_quat()
        return [float(q_flu[0]), float(q_flu[1]), float(q_flu[2]), float(q_flu[3])]

    @staticmethod
    def pose_ingress_ok(p, q) -> bool:
        # Degenerate ingress (NaN/Inf fields or a zero-norm quaternion) reaches R.from_quat in the
        # displacement/yaw gates and frame conversions -> scipy ValueError -> node death. Drop the
        # frame at the door.
        if not all(math.isfinite(v) for v in (p[0], p[1], p[2], q[0], q[1], q[2], q[3])):
            return False
        return (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]) > 1e-6

    def est_status_callback(self, msg):
        # Subscription is kept alive for the node's whole life so fusion detection can
        # re-fire if EKF2 ever drops and re-establishes EV fusion. (A destroy-on-first-True
        # here trapped ev_fusion_started False after a re-arm, bypassing the
        # velocity/displacement gates entirely.)
        if not self.ev_fusion_started:
            self.ev_fusion_started = msg.cs_ev_pos
            if self.ev_fusion_started:
                # Settle window should measure EKF2-fusion time, not FMU downtime.
                self.last_set_pose_time = self.get_clock().now()


    def visual_slam_status_callback(self, msg):
        self.vslam_status = msg.vo_state


    def vslam_check(self):
        # Busy-wedge watchdog: a SetSlamPose call whose future never completes (cuVSLAM died
        # mid-call) would leave vslam_busy True forever and kill the EV stream permanently -
        # nothing else clears it. Bounded here because this runs on every frame. The force-clear
        # RETIRES the seq token, so a late straggler response is dropped unconditionally - it can
        # neither clear a successor's vslam_busy nor bump the epoch at an arbitrary later time for
        # a pose whose commit was never observed. It also opens the bypass window: in the case the
        # watchdog exists for (cuVSLAM APPLIED the pose, only the response was lost) the pose HAS
        # moved, and judging the next frame against the frozen pre-re-seat baseline would read the
        # step as a jump and restart the loop. If the pose was NOT applied the rebase is a no-op.
        # The epoch is deliberately NOT bumped here - it stays confirmation-only.
        if self.vslam_busy:
            busy_s = (self.get_clock().now() - self._vslam_busy_since).nanoseconds * 1e-9
            if busy_s > self.set_pose_busy_timeout_s:
                self.get_logger().error(
                    f"SetSlamPose response never arrived ({busy_s:.1f}s > "
                    f"{self.set_pose_busy_timeout_s:.1f}s) - clearing vslam_busy (wedge guard)")
                self._seat_seq = (self._seat_seq + 1) & 0xFFFFFFFF
                self._last_reseat_commit = self.get_clock().now()
                self._reseat_bypass_frames = 0
                self.vslam_busy = False
        if self.vslam_status == 1 and not self.vslam_busy:
            return True
        else:
             return False


    def odom_velocity_gate(self, vslam_odom_msg: Odometry) -> bool:

        if (self.last_vslam_odom_msg.header.stamp.sec == 0 and 
            self.last_vslam_odom_msg.header.stamp.nanosec == 0):
            for field in Odometry.__slots__:
                setattr(self.last_vslam_odom_msg, field, getattr(vslam_odom_msg, field))
            return False

        current_odom_time = rclpy.time.Time.from_msg(vslam_odom_msg.header.stamp)
        previous_odom_time = rclpy.time.Time.from_msg(self.last_vslam_odom_msg.header.stamp)

        delta_time = (current_odom_time - previous_odom_time).nanoseconds * 1e-9
        if delta_time <= 0: return False

        delta_angle = self.min_quat_theta(vslam_odom_msg.pose.pose.orientation, 
                                          self.last_vslam_odom_msg.pose.pose.orientation)
        
        delta_position = self.calculate_3d_displacement(vslam_odom_msg.pose.pose, 
                                                        self.last_vslam_odom_msg.pose.pose)

        # No-motion short-circuit
        if delta_position < 1e-3 and delta_angle < np.radians(0.1):
            for field in Odometry.__slots__:
                setattr(self.last_vslam_odom_msg, field, getattr(vslam_odom_msg, field))
            return False

        linear_velocity = delta_position / delta_time               # meters/second
        angular_velocity = delta_angle / delta_time                 # rad/second

        # Quick debug output
        # self.get_logger().info(f"PREV POS: ({self.last_vslam_odom_msg.pose.pose.position.z}")
        # self.get_logger().info(f"CURR POS: ({vslam_odom_msg.pose.pose.position.z}")
        # self.get_logger().info(f"DELTA TIME: {delta_time}")
        # self.get_logger().info(f"DELTA POS: {delta_position}")
        # self.get_logger().info(f"LINEAR VELOCITY: {linear_velocity}")
        # self.get_logger().info(f"ANGULAR VELOCITY: {angular_velocity}")
        
        # The pos-delta AND-term catches slow teleports at normal cadence.
        jump = (linear_velocity >= self.lin_vel_gate or \
                angular_velocity >= self.ang_vel_gate or \
                (delta_position > self.VO_pos_delta_lim and delta_time > self.VO_rate_lim))

        if not jump:
            for field in Odometry.__slots__:
                setattr(self.last_vslam_odom_msg, field, getattr(vslam_odom_msg, field))
        else:
            self.get_logger().info(f"<<<<< VSLAM JUMP DETECTED >>>>>")

        return jump


    def reseat_bypass_gate(self, vslam_odom_msg: Odometry):
        """Returns (bypass_active, publishable).

        While inside the post-re-seat window the jump gate is BYPASSED and last_vslam_odom_msg is
        REBASED onto every incoming frame so the baseline tracks through the pose discontinuity a
        re-seat creates. A re-seat is purposeful by definition (origin or jump correction), so its
        step must never be re-judged as a spurious cuVSLAM jump. Without this the stale baseline
        makes the first post-re-seat frame a jump, which re-seats, which leaves the baseline stale
        again - a self-sustaining loop.

        WINDOW CLOSE is frame-count + wall-clock, not wall-clock alone: it needs BOTH
        vslam_stabilization_time elapsed AND >= RESEAT_BYPASS_MIN_FRAMES frames stamped after the
        commit rebased into the baseline. When the stream is degraded a
        pure wall-clock window can expire with ZERO admitted frames, re-arming the loop it exists
        to break. RESEAT_BYPASS_CEILING_S caps it so a dead stream cannot pin the bypass open.

        PUBLISHABLE is False for frames stamped BEFORE the commit: such an in-flight frame may
        still carry the pre-re-seat pose, and publishing it with the already-bumped reset_counter
        makes EKF2 re-anchor height onto the UNCORRECTED EV z and then eat the full correction as
        an unflagged step. The frame still rebases the baseline (the loop-break needs that); it is
        only withheld from EV.

        Accepted risk: a REAL cuVSLAM jump inside the window is not merely missed - it is rebased
        into the baseline and invisible to the gate afterwards. Bounded by the window (nominally
        ~vslam_stabilization_time, hard-capped at RESEAT_BYPASS_CEILING_S), which only opens at a
        committed re-seat, the moment cuVSLAM is freshest against EKF2.
        """
        if self._last_reseat_commit is None:
            return False, True
        elapsed = (self.get_clock().now() - self._last_reseat_commit).nanoseconds * 1e-9
        # Close is a ONE-SHOT event: _last_reseat_commit is cleared so the ceiling warn cannot
        # repeat for the rest of the flight (the baseline self-heals on the first post-close frame
        # via the not-jump refresh or a fresh re-seat).
        if elapsed >= RESEAT_BYPASS_CEILING_S:
            if self._reseat_bypass_frames < RESEAT_BYPASS_MIN_FRAMES:
                self.get_logger().warn(
                    f"RESEAT BYPASS: ceiling {RESEAT_BYPASS_CEILING_S:.1f}s hit with only "
                    f"{self._reseat_bypass_frames} post-commit frames rebased - baseline may "
                    f"still be pre-re-seat")
            self._last_reseat_commit = None
            return False, True
        if (elapsed >= self.vslam_stabilization_time
                and self._reseat_bypass_frames >= RESEAT_BYPASS_MIN_FRAMES):
            self._last_reseat_commit = None
            return False, True
        for field in Odometry.__slots__:
            setattr(self.last_vslam_odom_msg, field, getattr(vslam_odom_msg, field))
        # clock_type must match the commit stamp's (node clock): cross-clock subtraction raises
        # TypeError out of the executor (same defense as the FMU stamp-skew path).
        stamp_t = rclpy.time.Time.from_msg(vslam_odom_msg.header.stamp,
                                           clock_type=self._last_reseat_commit.clock_type)
        fresh = (stamp_t - self._last_reseat_commit).nanoseconds > 0
        if fresh:
            self._reseat_bypass_frames += 1
        return True, fresh


    def _vo_health_update(self):
        healthy = not (self._burst_unhealthy or self._silence_unhealthy)
        if healthy != self._vo_healthy:
            self._vo_healthy = healthy
            self.pub_vo_healthy_.publish(Bool(data=healthy))

    def _ev_silence_check(self):
        # Publish-silence observable: every other monitor watches the pipe's INPUT (cuVSLAM stamps,
        # /visual_slam/status, camera rates). A burst blackout or a bypass wedge leaves EKF2
        # EV-starved while all of them read healthy. Alarm only when frames are ARRIVING and
        # fusion is expected; EKF2's own EV de-latch covers input silence.
        now = self.get_clock().now()
        frames_flowing = (self._last_vslam_rx is not None
                          and (now - self._last_vslam_rx).nanoseconds * 1e-9 < 2.0)
        pub_age = ((now - self._last_ev_pub).nanoseconds * 1e-9
                   if self._last_ev_pub is not None else 0.0)
        silent = (frames_flowing and self.ev_fusion_started
                  and pub_age > self.ev_silence_max_s)
        if silent != self._silence_unhealthy:
            self._silence_unhealthy = silent
            if silent:
                self.get_logger().error(
                    f"EV PUBLISH SILENCE: {pub_age:.1f}s since last EV output with cuVSLAM "
                    f"frames flowing (limit {self.ev_silence_max_s:.1f}s)")
            else:
                self.get_logger().info("EV publish silence cleared - output flowing")
            self._vo_health_update()

    def _reseat_burst_eval(self) -> bool:
        # Prune the rolling window and keep the latched /reactor/vo_healthy verdict in sync.
        # Returns True while the budget is exhausted (jump re-seats blocked). Never touches
        # vslam_busy or the reset epoch - blocking a re-seat is not a re-seat.
        now_s = self.get_clock().now().nanoseconds * 1e-9
        while self._reseat_burst and (now_s - self._reseat_burst[0]) > self.reseat_burst_window_s:
            self._reseat_burst.popleft()
        # < not <=: reseat_burst_max COMMITTED re-seats are allowed in the window, the next
        # attempt is blocked.
        healthy = len(self._reseat_burst) < self.reseat_burst_max
        if healthy == self._burst_unhealthy:
            self._burst_unhealthy = not healthy
            self._vo_health_update()
            # Distinct single-severity log call sites (rclpy pins severity per call site).
            if not healthy:
                self.get_logger().error(
                    f"RE-SEAT BURST: {len(self._reseat_burst)} jump re-seats in "
                    f"{self.reseat_burst_window_s:.0f}s (budget {self.reseat_burst_max}) - cuVSLAM "
                    f"is not recoverable by re-seating; suspending jump re-seats")
            else:
                self.get_logger().info(
                    f"RE-SEAT BURST cleared: {len(self._reseat_burst)} in "
                    f"{self.reseat_burst_window_s:.0f}s - jump re-seats re-enabled")
        return not healthy


    def _reseat_on_jump(self):
        # Sole entry point for JUMP re-seats: rate-limited by the burst window. Re-seating at
        # multi-Hz cannot recover cuVSLAM, it only feeds EKF2 a reset storm. The reactor raises
        # the flag and stops; it never commands a flight action.
        if self._reseat_burst_eval():
            return
        if self.set_slam_pose():
            self._reseat_burst.append(self.get_clock().now().nanoseconds * 1e-9)
            self._reseat_burst_eval()


    # Potentially depreciate
    def odom_temporal_reset_gate(self):
        time_delta = (self.get_clock().now() - self.last_set_pose_time).nanoseconds * 1e-9
        if time_delta < self.vslam_stabilization_time and not self.init_flag:
            return True
        return False
       

    def odom_displacement_gate(self, vslam_odom_msg: Odometry) -> bool:
        # If VSLAM is reset, check if it is within bounds
        if self.new_set_pose_call:
            current_odom_time = rclpy.time.Time.from_msg(vslam_odom_msg.header.stamp)
            drone_odom_msg = self.sync_msg(current_odom_time, self._drone_odom_cache)

            if drone_odom_msg is None:
                # No per-frame log: this is polled every frame during the settle window.
                return True

            # Dedicated settle-exit tolerances (align_yaw_deg / align_pos_m), separate from the jump
            # gate: yaw is gated on its own axis, position on the combined 3D norm.
            yaw_err = self.yaw_delta(drone_odom_msg.pose.pose.orientation,
                                     vslam_odom_msg.pose.pose.orientation)
            pos_err = self.calculate_3d_displacement(vslam_odom_msg.pose.pose,
                                                     drone_odom_msg.pose.pose)
            if yaw_err >= self.align_yaw or pos_err >= self.align_pos_m:
                # Still misaligned (EKF2 not yet converged). Polled per frame during the settle
                # window, so do not log here; the caller warns once per (re)injection.
                return True
            self.new_set_pose_call = False
            self.get_logger().info(
                f"<<<<< SETTLE EXIT ({self._settle_owner}): EKF2 aligned "
                f"(yaw {np.degrees(yaw_err):.2f}deg <= {self.align_yaw_deg:.1f}, "
                f"pos {pos_err:.3f}m <= {self.align_pos_m:.2f}) - streaming normally >>>>>")

        return False


    def set_slam_pose(self, init=False):
        last_drone_odom_msg = self._drone_odom_cache.getLast()
        if not last_drone_odom_msg:
            return False

        # Freshness gate: skip stale (pre-reboot) PX4 samples; caller retries each frame until fresh.
        age = (self.get_clock().now() - self._last_px4_rx).nanoseconds / 1e9
        if age > self.set_pose_max_odom_age:
            return False

        self.last_set_pose_time = self.get_clock().now()
        req = SetSlamPose.Request()

        # Position: Converted vehicle position in FLU frame
        if init:
            req.pose.position.x = 0.0
            req.pose.position.y = 0.0
            req.pose.position.z = 0.0
        else:
            req.pose.position.x = last_drone_odom_msg.pose.pose.position.x
            req.pose.position.y = last_drone_odom_msg.pose.pose.position.y
            req.pose.position.z = last_drone_odom_msg.pose.pose.position.z

        # Orientation: Converted vehicle quaternion in FLU [x, y, z, w].
        # On init (pre-takeoff datum) zero YAW as well as position: mag is disabled so the
        # heading datum is arbitrary (POSE_FRAME_FRD = "arbitrary heading reference"), and a
        # clean 0-yaw origin removes the re-anchor jump EKF2 would otherwise reject. Roll/pitch
        # are kept from the FMU so the frame stays gravity-aligned. Non-init (in-flight) re-seats
        # keep the FMU's full orientation, so the re-anchor vs EKF2's current estimate stays ~zero.
        _q_flu = last_drone_odom_msg.pose.pose.orientation
        if init:
            # UPPERCASE 'ZYX' (intrinsic z-y'-x''): element 0 is the WORLD yaw, so zeroing it
            # composes Rz(-yaw) on the WORLD side and the map frame stays gravity-aligned.
            # Lowercase 'zyx' (extrinsic) puts the yaw change on the BODY side, tilting cuVSLAM's
            # map off gravity by 2*tilt*sin(yaw/2) on a sloped pad, which cross-couples horizontal
            # travel into EV height.
            _rpy = R.from_quat([_q_flu.x, _q_flu.y, _q_flu.z, _q_flu.w]).as_euler('ZYX')
            _rpy[0] = 0.0  # zero yaw, keep pitch/roll
            _q_zeroed = R.from_euler('ZYX', _rpy).as_quat()
            req.pose.orientation.x = float(_q_zeroed[0])
            req.pose.orientation.y = float(_q_zeroed[1])
            req.pose.orientation.z = float(_q_zeroed[2])
            req.pose.orientation.w = float(_q_zeroed[3])
        else:
            req.pose.orientation.x = _q_flu.x
            req.pose.orientation.y = _q_flu.y
            req.pose.orientation.z = _q_flu.z
            req.pose.orientation.w = _q_flu.w
        
        _q = req.pose.orientation
        _yaw = np.degrees(R.from_quat([_q.x, _q.y, _q.z, _q.w]).as_euler('ZYX')[0])
        _msg = (f"init={init} "
                f"pos=({req.pose.position.x:.3f},{req.pose.position.y:.3f},{req.pose.position.z:.3f}) "
                f"yaw={_yaw:.1f}deg src_odom_age={age:.4f}s")
        # Two DISTINCT log call sites, never one line with switched severity: rclpy pins a call
        # site's severity on first use and RAISES on a change, so a `warn if init else info` emit
        # kills the node on the first jump re-seat after an origin seat.
        if init:
            self.get_logger().warn(f">>> SET ORIGIN (settling) <<< {_msg}")
        else:
            self.get_logger().info(f">>> VSLAM SET POSE <<< {_msg}")
        # Stamp BEFORE raising busy: the busy-wedge watchdog reads (busy, since) unlocked from
        # another path; the reverse order can pair busy=True with the PREVIOUS seat's stamp and
        # fire an instant spurious force-clear.
        self._vslam_busy_since = self.get_clock().now()
        self.vslam_busy = True
        self._seat_is_init = init
        self.new_set_pose_call = True
        # Settle ownership: only an 'origin' settle may escalate to an origin re-injection on
        # settle timeout (slam_odom_callback). A 'reseat' settle that fails to align is abandoned
        # with an error - set_slam_pose(True) zeroes position AND yaw, and doing that mid-flight
        # destroys the heading datum (no yaw rebase exists downstream).
        self._settle_owner = 'origin' if init else 'reseat'

        # The reset epoch is bumped in service_response_callback on SUCCESS, not here: bumping
        # before the re-seat completes lets vio_transform stamp the new reset_counter onto a
        # pre-reseat pose (EKF2 re-anchors onto stale data), and a failed re-seat would burn an
        # epoch with no pose change. vslam_busy suppresses EV publishing until the callback runs.
        # The seq token pins the response to THIS re-seat: after a busy-wedge force-clear starts
        # a successor, the straggler's late response must not clear the successor's vslam_busy or
        # bump an epoch for a pose that was never committed.
        # Capture the incremented value into a LOCAL and bind that local into the closure:
        # `_seq=self._seat_seq` as a lambda default RE-READS the attribute at binding time, so a
        # concurrent bump between the store and the bind could hand this response the successor's
        # token.
        seq = self._seat_seq = (self._seat_seq + 1) & 0xFFFFFFFF
        future = self.set_slam_pose_client.call_async(req)
        future.add_done_callback(
            lambda f, _seq=seq: self.service_response_callback(f, _seq))
        return True


    def set_slam_pose_callback(self, request, response):   
        if self.vslam_check(): 
            self.set_slam_pose(True)
            response.success = True
        else:
            response.success = False
        return response


    def publish_vslam_to_px4(self, vslam_odom_msg):
        self._last_ev_pub = self.get_clock().now()
        # if not self.odom_temporal_reset_gate(): # Potentially depreciate 
        self.pub_filtered_odom_.publish(vslam_odom_msg)


    def slam_odom_callback(self, vslam_odom_msg):
        _p = vslam_odom_msg.pose.pose.position
        _q = vslam_odom_msg.pose.pose.orientation
        if not self.pose_ingress_ok((_p.x, _p.y, _p.z), (_q.x, _q.y, _q.z, _q.w)):
            self.get_logger().error("VSLAM ingress: non-finite/zero-norm pose dropped",
                                    throttle_duration_sec=1.0)
            return
        self._last_vslam_rx = self.get_clock().now()
        if self._reseat_burst:
            # Decay the burst window even when no jump fires, so /reactor/vo_healthy re-latches
            # once the storm ends. Guarded because the deque is empty in normal flight.
            self._reseat_burst_eval()
        if self.vslam_check():
            if self.ev_fusion_started is False:
                if self.init_flag:
                    if self.set_slam_pose(True):
                        self.init_flag = False
                else:
                    # Keep the jump-gate baseline live on the pre-fusion path: this path streams
                    # without ever entering the gate, so without the rebase the FIRST fused frame
                    # is judged against however old a pose the baseline still holds - a spurious
                    # jump re-seat (plus EKF2 reset) immediately at fusion start.
                    for field in Odometry.__slots__:
                        setattr(self.last_vslam_odom_msg, field, getattr(vslam_odom_msg, field))
                    self.publish_vslam_to_px4(vslam_odom_msg)
            else:
                bypass, bypass_fresh = self.reseat_bypass_gate(vslam_odom_msg)
                if bypass or not self.odom_velocity_gate(vslam_odom_msg):
                    if bypass and not bypass_fresh:
                        # In-flight frame stamped BEFORE the commit: the baseline is already
                        # rebased (loop-break), but the pose may predate the re-seat, and
                        # publishing it would carry the bumped reset_counter - EKF2 re-anchors
                        # height onto the uncorrected EV z, then eats the full correction as an
                        # unflagged step. Withheld from EV only.
                        self.get_logger().warn(
                            "RESEAT BYPASS: pre-commit frame withheld from EV",
                            throttle_duration_sec=1.0)
                    elif not self.odom_displacement_gate(vslam_odom_msg):
                        # Aligned (EKF2 converged onto the origin) -> stream normally.
                        self.publish_vslam_to_px4(vslam_odom_msg)
                    elif not self.init_flag:
                        # Misaligned: EKF2 has not yet converged onto the freshly-injected origin.
                        # Do NOT re-seat every frame -- rapid reset_counter bumps stop EKF2 ever
                        # converging. Keep INJECTING the stream for a settle window so EKF2 can align;
                        # only re-inject the origin (one reset) if the window elapses without alignment.
                        settle_dt = (self.get_clock().now() - self.last_set_pose_time).nanoseconds * 1e-9
                        if settle_dt < self.set_origin_settle_time:
                            self.publish_vslam_to_px4(vslam_odom_msg)
                        elif self._settle_owner != 'origin':
                            # A jump-re-seat settle that fails to align must NEVER escalate to
                            # set_slam_pose(True): that is an ORIGIN injection - position zeroed
                            # to (0,0,0) AND yaw zeroed - and in flight it destroys the heading
                            # datum (EKF2 resets rebase z/xy downstream, no yaw rebase exists).
                            # Abandon alignment: the epoch is already committed, EKF2 re-anchors
                            # on its own; keep streaming.
                            self.get_logger().error(
                                f"SETTLE ({self._settle_owner}): EKF2 not aligned within "
                                f"{self.set_origin_settle_time:.1f}s - abandoning alignment "
                                f"(origin re-injection is origin-settle only)",
                                throttle_duration_sec=1.0)
                            self.new_set_pose_call = False
                            self.publish_vslam_to_px4(vslam_odom_msg)
                        else:
                            # set_slam_pose's freshness gate no-ops when PX4 odom is stale (FMU down),
                            # so skip the warn+call this frame rather than repeatedly log an action that never runs.
                            px4_age = (self.get_clock().now() - self._last_px4_rx).nanoseconds / 1e9
                            if px4_age <= self.set_pose_max_odom_age:
                                self.get_logger().warn(
                                    f"SET ORIGIN: EKF2 not aligned within {self.set_origin_settle_time:.1f}s "
                                    f"settle window - re-injecting origin",
                                    throttle_duration_sec=1.0)
                                self.set_slam_pose(True)
                else:
                    self._reseat_on_jump()
        elif self.new_set_pose_call:
            # Not tracking / re-seat in flight while a settle is pending: the settle countdown must
            # measure healthy STREAMING time only. Without this rebase a >settle-window tracking
            # dropout banks the whole countdown and the first frame after resume lands straight in
            # the settle-timeout branch.
            self.last_set_pose_time = self.get_clock().now()


def main(args=None):
    rclpy.init(args=args)
    tracker = vslam_reactor()
    # 3 threads for 3 MutuallyExclusive groups (vslam_cbg = VO frames + all re-seat machinery,
    # gen_processing_cbg = FMU ingress/status, default = tf listener): with 2 threads two busy
    # groups starve the third, and response-side starvation is extra EV silence.
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(tracker)
    try:
        executor.spin()
    except KeyboardInterrupt:
        tracker.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        tracker.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()