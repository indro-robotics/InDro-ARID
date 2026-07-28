#!/usr/bin/env python
import math
import rclpy
import numpy as np

CADENCE_ANOMALY_CEILING_S = 5.0   # stamp gap beyond this = stamp anomaly, never a gate trigger
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
        self._seat_seq = 0   # re-seat generation: straggler SetSlamPose responses must not touch the successor's state
        self.new_set_pose_call = False
        self.ev_fusion_started = False                      # PX4 EV fusion
        self._vio_reset_epoch = 0                           # bumped on each committed re-seat -> EKF2 reset_counter
        self.last_set_pose_time = self.get_clock().now()

        self.fmu_local_position = Vector3Stamped()
        self.last_vslam_odom_msg = Odometry()



        # Cadence-health gate: withholds VO from EKF2 while the cuVSLAM frame cadence is broken
        # (camera-swap starvation produces smooth-but-stale poses that vo_state and the jump gate
        # cannot see; fusing them causes a lag -> failover -> reset cascade). Dedicated stamp var:
        # last_vslam_odom_msg's stamp can be frozen by the jump path, so it is NOT a valid cadence
        # reference. Nothing on this platform consumes /reactor/cadence_gated (no failsafe
        # consumer), so a withhold is unbounded: EKF2 rides ARK flow + rangefinder until
        # cadence proves nominal again.
        self._last_cadence_stamp = None       # rclpy Time of previous VO header.stamp
        self._cadence_gated = False           # currently withholding
        self._cadence_nominal_run = 0         # consecutive nominal-cadence samples while gated
        self._cadence_over_run = 0            # consecutive lesser-gap samples while ungated (two-tier engage)
        self._cadence_gate_t0 = None          # node-clock time the current gate engaged
        self._cadence_gated_frames = 0        # frames seen during the current gated window
        self._cadence_gate_count = 0          # cumulative gate events (telemetry)
        self._cadence_trigger_gap_ms = 0.0    # gap that engaged the current gate (silence accounting)

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

        # Cadence-gate event counter (latched): post-flight forensics can read how many times the
        # gate engaged without scraping logs. Bumped once per gate ENGAGEMENT, not per frame.
        self.pub_cadence_gate_count_ = self.create_publisher(UInt32,
                                                             '/reactor/cadence_gate_count',
                                                             qos_profile=self.qos_transient,
                                                             callback_group=self.vslam_cbg)
        self.pub_cadence_gate_count_.publish(UInt32(data=0))

        # Live gate state (latched): True while the gate is withholding. /visual_slam/status stays
        # healthy during starvation, so without this an observer cannot see EV being withheld at all.
        self.pub_cadence_gated_ = self.create_publisher(Bool,
                                                        '/reactor/cadence_gated',
                                                        qos_profile=self.qos_transient,
                                                        callback_group=self.vslam_cbg)
        self.pub_cadence_gated_.publish(Bool(data=False))


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
        self.set_slam_pose_client = self.create_client(SetSlamPose, 'visual_slam/set_slam_pose')
        
        self.trigger_slam_pose_service = self.create_service(Trigger, 'visual_slam/set_reactor_pose',
                                                             self.set_slam_pose_callback,
                                                             callback_group=self.gen_processing_cbg)

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
            ('vslam_stabilization_time', 0.5),
            ('lin_vel_gate', 15.0),
            ('ang_vel_gate_dps', 900.0),
            ('VO_rate_lim', 0.20),
            ('VO_pos_delta_lim', 0.4),
            ('sync_cache_sz', 300),
            ('align_yaw_deg', 2.0),
            ('align_pos_m', 0.10),
            ('set_pose_max_odom_age', 0.010),
            ('set_origin_settle_time', 10.0),
            ('fmu_stamp_max_skew_s', 0.5),
            ('cadence_gate_s', 0.15),
            ('cadence_gate_hard_s', 0.40),
            ('cadence_gate_sustained_samples', 5),
            ('cadence_release_samples', 5),
            ('set_pose_busy_timeout_s', 5.0),
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

    def _cadence_update(self, vslam_odom_msg):
        """Track VO header.stamp cadence and maintain the gate state. Called on EVERY frame
        arriving at slam_odom_callback, BEFORE any processing guard, so busy/vo_state windows keep
        the bookkeeping warm and cannot fabricate a gap on resume.

        Stamps are camera-frame acquisition time (cuVSLAM override_publishing_stamp=false), so
        consecutive-stamp deltas ARE the frame cadence; a starvation gap surfaces as one large
        delta on the next sample. Stamp deltas are a LOWER BOUND on the silence EKF2 experiences
        (EKF2 de-latches on ARRIVAL silence; a pure arrival stall with contiguous stamps is
        invisible here and is handled natively by EKF2's own 400 ms de-latch + clean re-anchor).
        The gate's protected class is degraded-cadence starvation bursts with advancing stamps.
        It is severity reduction, not prevention.
        """
        stamp = rclpy.time.Time.from_msg(vslam_odom_msg.header.stamp)
        prev = self._last_cadence_stamp
        self._last_cadence_stamp = stamp
        if prev is None:
            return
        dt = (stamp - prev).nanoseconds * 1e-9
        if dt <= 0:
            # Stamp anomaly, not a gap; break the sustained streak (not evidence).
            self._cadence_over_run = 0
            return
        if dt > CADENCE_ANOMALY_CEILING_S:
            # A gap this large is a stamp-regime anomaly (rogue future stamp, clock jump, cuVSLAM
            # restart), not starvation: engaging on it would wedge the gate until wall-clock
            # overtakes the rogue stamp. Reseed and move on; genuine outages this long are already
            # EKF2-de-latched and jump-gate territory.
            self._cadence_over_run = 0
            return

        now = self.get_clock().now()
        if dt > self.cadence_gate_s:
            if not self._cadence_gated:
                # Two-tier engage: one hard gap >= cadence_gate_hard_s (EKF2's own arrival
                # de-latch point; lone 150-400 ms gaps are valid-late poses it absorbs, and the
                # gate's job past 400 is preventing a re-latch onto the stale burst), or
                # sustained_samples consecutive lesser gaps. Jump gate separate, always live.
                hard = dt >= self.cadence_gate_hard_s
                if not hard:
                    self._cadence_over_run += 1
                if hard or self._cadence_over_run >= self.cadence_gate_sustained_samples:
                    self._cadence_gated = True
                    self._cadence_gate_t0 = now
                    self._cadence_trigger_gap_ms = dt * 1000.0
                    self._cadence_gated_frames = 0
                    self._cadence_gate_count += 1
                    self._cadence_over_run = 0
                    self.pub_cadence_gate_count_.publish(UInt32(data=self._cadence_gate_count))
                    self.pub_cadence_gated_.publish(Bool(data=True))
                    self.get_logger().warn(
                        f"CADENCE GATE: {'HARD gap' if hard else 'sustained degraded cadence'} "
                        f"{dt*1000:.0f}ms (gate {self.cadence_gate_s*1000:.0f}ms, hard "
                        f"{self.cadence_gate_hard_s*1000:.0f}ms) "
                        f"- withholding EV (event #{self._cadence_gate_count})",
                        throttle_duration_sec=1.0)
                return
            self._cadence_nominal_run = 0
            self._cadence_gated_frames += 1
            self.get_logger().warn(
                f"CADENCE GATE: VO stamp gap {dt*1000:.0f}ms > {self.cadence_gate_s*1000:.0f}ms "
                f"- withholding EV (event #{self._cadence_gate_count})",
                throttle_duration_sec=1.0)
        elif self._cadence_gated:
            self._cadence_over_run = 0
            self._cadence_nominal_run += 1
            self._cadence_gated_frames += 1
            if self._cadence_nominal_run >= self.cadence_release_samples:
                held_ms = (now - self._cadence_gate_t0).nanoseconds * 1e-6
                self._cadence_release(held_ms)
        else:
            # Clean delta while ungated: a lesser-gap streak must be CONSECUTIVE -
            # any nominal sample breaks it.
            self._cadence_over_run = 0

        # NO timed escape: the gate never releases on a clock - only proven-nominal cadence
        # readmits VO. A degraded stream stays withheld indefinitely; EKF2 coasts on flow+rng
        # after its native 400 ms EV de-latch, and if ALL horizontal aiding is lost PX4's own
        # ladder takes over (1 s aid-timeout -> inertial dead-reckon -> EKF2_NOAID_TOUT 5 s ->
        # local position invalid -> commander position-loss failsafe). Feeding one degraded
        # sample per window would re-anchor EKF2 onto lagged poses repeatedly.
        # Prolonged-withhold visibility: /reactor/cadence_gated stays latched True.

    def _cadence_release(self, held_ms):
        total_ms = held_ms + self._cadence_trigger_gap_ms
        self._cadence_gated = False
        self._cadence_nominal_run = 0
        self.pub_cadence_gated_.publish(Bool(data=False))
        note = (" - EV silence exceeded EKF2's 400ms de-latch: expect a clean re-anchor reset "
                "on resume" if total_ms > 400.0 else "")
        self.get_logger().info(
            f"CADENCE GATE: released after {held_ms:.0f}ms hold "
            f"(+{self._cadence_trigger_gap_ms:.0f}ms trigger gap = {total_ms:.0f}ms EV silence), "
            f"{self._cadence_gated_frames} frames withheld (event #{self._cadence_gate_count}){note}")
        # The releasing frame streams in this same callback pass.

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
                self._vio_reset_epoch = (self._vio_reset_epoch + 1) & 0xFF
                self.pub_vio_reset_epoch_.publish(UInt8(data=self._vio_reset_epoch))
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
        y1 = R.from_quat([q1_orientation.x, q1_orientation.y, q1_orientation.z, q1_orientation.w]).as_euler('zyx')[0]
        y2 = R.from_quat([q2_orientation.x, q2_orientation.y, q2_orientation.z, q2_orientation.w]).as_euler('zyx')[0]
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
        # nothing else clears it. Bounded here because this runs on every frame. A late straggler
        # response after the force-clear is dropped by the _seat_seq mismatch check in
        # service_response_callback, so it can neither clear a successor's vslam_busy nor burn an
        # epoch for a pose that was never committed.
        if self.vslam_busy:
            busy_s = (self.get_clock().now() - self._vslam_busy_since).nanoseconds * 1e-9
            if busy_s > self.set_pose_busy_timeout_s:
                self.get_logger().error(
                    f"SetSlamPose response never arrived ({busy_s:.1f}s > "
                    f"{self.set_pose_busy_timeout_s:.1f}s) - clearing vslam_busy (wedge guard)")
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
        
        jump = (linear_velocity >= self.lin_vel_gate or \
                angular_velocity >= self.ang_vel_gate or \
                (delta_position > self.VO_pos_delta_lim and delta_time > self.VO_rate_lim))

        if not jump:
            for field in Odometry.__slots__:
                setattr(self.last_vslam_odom_msg, field, getattr(vslam_odom_msg, field))
        else:
            self.get_logger().info(f"<<<<< VSLAM JUMP DETECTED >>>>>")

        return jump
    

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
                f"<<<<< SET ORIGIN: EKF2 aligned (yaw {np.degrees(yaw_err):.2f}deg <= {self.align_yaw_deg:.1f}, "
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
            _rpy = R.from_quat([_q_flu.x, _q_flu.y, _q_flu.z, _q_flu.w]).as_euler('zyx')
            _rpy[0] = 0.0  # zero yaw, keep pitch/roll
            _q_zeroed = R.from_euler('zyx', _rpy).as_quat()
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
        _yaw = np.degrees(R.from_quat([_q.x, _q.y, _q.z, _q.w]).as_euler('zyx')[0])
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
        self.new_set_pose_call = True

        # The reset epoch is bumped in service_response_callback on SUCCESS, not here: bumping
        # before the re-seat completes lets vio_transform stamp the new reset_counter onto a
        # pre-reseat pose (EKF2 re-anchors onto stale data), and a failed re-seat would burn an
        # epoch with no pose change. vslam_busy suppresses EV publishing until the callback runs.
        # The seq token pins the response to THIS re-seat: after a busy-wedge force-clear starts
        # a successor, the straggler's late response must not clear the successor's vslam_busy or
        # bump an epoch for a pose that was never committed.
        self._seat_seq = (self._seat_seq + 1) & 0xFFFFFFFF
        future = self.set_slam_pose_client.call_async(req)
        future.add_done_callback(
            lambda f, _seq=self._seat_seq: self.service_response_callback(f, _seq))
        return True


    def set_slam_pose_callback(self, request, response):   
        if self.vslam_check(): 
            self.set_slam_pose(True)
            response.success = True
        else:
            response.success = False
        return response


    def publish_vslam_to_px4(self, vslam_odom_msg):
        # if not self.odom_temporal_reset_gate(): # Potentially depreciate 
        self.pub_filtered_odom_.publish(vslam_odom_msg)


    def slam_odom_callback(self, vslam_odom_msg):
        _p = vslam_odom_msg.pose.pose.position
        _q = vslam_odom_msg.pose.pose.orientation
        if not self.pose_ingress_ok((_p.x, _p.y, _p.z), (_q.x, _q.y, _q.z, _q.w)):
            # Dropped BEFORE cadence bookkeeping: a burst of degenerate frames reads as a stamp
            # gap and engages the cadence gate, which is the correct degraded-stream response.
            self.get_logger().error("VSLAM ingress: non-finite/zero-norm pose dropped",
                                    throttle_duration_sec=1.0)
            return
        # Cadence bookkeeping runs on EVERY frame BEFORE any guard: busy/vo_state windows must not
        # freeze the stamp reference (a stale reference would read the whole window as one giant
        # gap and spuriously gate the first resumed frame).
        self._cadence_update(vslam_odom_msg)
        if self.vslam_check():
            if self.ev_fusion_started is False:
                if self.init_flag:
                    if self.set_slam_pose(True):
                        self.init_flag = False
                else:
                    self.publish_vslam_to_px4(vslam_odom_msg)
            elif self._cadence_gated:
                # ENFORCED WITHHOLD: starvation-cadence VO must not reach EKF2 (smooth-but-stale
                # poses pass vo_state AND the jump gate). ALL settles are withheld with the settle
                # countdown paused (no init bypass - the settle-timeout re-inject can fire mid-
                # flight and would disable the gate around a fresh airborne origin). NEVER touches
                # vslam_busy or the reset epoch - a withhold is not a re-seat.
                if self.new_set_pose_call:
                    # Settle in progress while gated: rebase the countdown so gate-induced
                    # starvation cannot fire the settle-timeout re-inject; alignment evaluation
                    # resumes on gate release.
                    self.last_set_pose_time = self.get_clock().now()
                if self.odom_velocity_gate(vslam_odom_msg):
                    # Jump detection stays LIVE during the gate: a genuine cuVSLAM jump inside a
                    # gated window must re-seat exactly as it would ungated (the non-jump path
                    # refreshes last_vslam_odom_msg so the release-edge delta is computed against
                    # the newest gated pose).
                    self.set_slam_pose()
                # else: frame withheld - no publish
            elif not self.odom_velocity_gate(vslam_odom_msg):
                if not self.odom_displacement_gate(vslam_odom_msg):
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
                self.set_slam_pose()


def main(args=None):
    rclpy.init(args=args)
    tracker = vslam_reactor()
    executor = MultiThreadedExecutor(num_threads=2)
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