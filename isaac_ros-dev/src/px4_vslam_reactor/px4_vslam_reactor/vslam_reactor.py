#!/usr/bin/env python
import os
import yaml
import rclpy
import numpy as np
import message_filters
from rclpy.node import Node
from std_msgs.msg import Bool
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
from ament_index_python.packages import get_package_share_directory


class vslam_reactor(Node):
    def __init__(self): 
        super().__init__('vslam_reactor_node')

        # REACTOR CONTROL STATE
        self.init_flag = True
        self.vslam_status = 0
        self.vslam_busy = False
        self.new_set_pose_call = False
        self.ev_fusion_started = False                      # PX4 EV fusion
        self.last_set_pose_time = self.get_clock().now()

        self.fmu_local_position = Vector3Stamped()
        self.last_vslam_odom_msg = Odometry()

        # TUNABLES — loaded from config/reactor_conf.yaml (installed to the
        # package's share dir by setup.py). See that file for per-parameter
        # descriptions. Hardcoded defaults below are used only as fallbacks
        # if a key is missing from the YAML.
        cfg = self._load_reactor_conf()
        self.vslam_stabilization_time = cfg.get('vslam_stabilization_time', 0.5)
        self.lin_vel_gate             = cfg.get('lin_vel_gate',             15.0)
        self.ang_vel_gate             = cfg.get('ang_vel_gate',             float(np.pi * 5))
        self.VO_rate_lim              = cfg.get('VO_rate_lim',              0.20)
        self.VO_pos_delta_lim         = cfg.get('VO_pos_delta_lim',         0.4)
        self.sync_cache_sz            = cfg.get('sync_cache_sz',            300)
        self.quat_delta_theta         = cfg.get('quat_delta_theta',         float(np.radians(5.0)))
        self.displacement_delta       = cfg.get('displacement_delta',       0.25)

        self.fmu_lockout = False
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

        self.sub_fmu_lockout = self.create_subscription(Bool,
                                                        '/px4_state_machine/fmu_lockout',
                                                        self.fmu_lockout_callback,
                                                        qos_profile=self.qos_transient,
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
        while not self.set_slam_pose_client.wait_for_service(): pass

        ### CALLBACK REGISTRATIONS #################################################################
        self._slam_odom_sub.registerCallback(self.slam_odom_callback)


    def _load_reactor_conf(self):
        """Load config/reactor_conf.yaml from this package's share dir.
        Returns the parameter dict, or {} if the file is missing/malformed
        (the caller falls back to hardcoded defaults)."""
        try:
            cfg_path = os.path.join(
                get_package_share_directory('px4_vslam_reactor'),
                'config', 'reactor_conf.yaml')
            with open(cfg_path, 'r') as f:
                data = yaml.safe_load(f) or {}
            params = data.get('vslam_reactor', {}).get('ros__parameters', {}) or {}
            self.get_logger().info(
                'Loaded reactor_conf.yaml (%d tunables)' % len(params))
            return params
        except FileNotFoundError:
            self.get_logger().warn(
                'reactor_conf.yaml not found — using built-in defaults')
            return {}
        except Exception as e:
            self.get_logger().error(
                'Failed to parse reactor_conf.yaml (%s) — using defaults' % e)
            return {}


    def px4_odom_callback(self, msg):

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
        map_px4_t.header.stamp = rclpy.time.Time(nanoseconds=msg.timestamp * 1000).to_msg()
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
        drone_odom_msg.header.stamp = rclpy.time.Time(nanoseconds=msg.timestamp * 1000).to_msg()
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

    def fmu_lockout_callback(self, msg):
        self.fmu_lockout = msg.data
        self.ev_fusion_started = False
        self.init_flag = True

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
    

    def service_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.vslam_busy = False
            else:
                self.get_logger().error(f"SetSlamPose failure: {response.message}")
        except Exception as e:
            self.get_logger().error(f"SetSlamPose call failed: {str(e)}")


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
    
    def position_frd_to_flu(self, pos_frd):
        return np.array([pos_frd[0],
                        -pos_frd[1],
                        -pos_frd[2]])

    def quat_frd_to_flu(self, q_frd):
        R_body_world_frd = R.from_quat(q_frd)
        R_body_world_flu = self.R_FRD_TO_FLU * R_body_world_frd * self.R_FRD_TO_FLU.inv()
        q_flu = R_body_world_flu.as_quat()
        return [float(q_flu[0]), float(q_flu[1]), float(q_flu[2]), float(q_flu[3])]

    def est_status_callback(self, msg):
        if not self.ev_fusion_started:
            self.ev_fusion_started = msg.cs_ev_pos
            if self.ev_fusion_started:
                self.destroy_subscription(self.est_status_sub)


    def visual_slam_status_callback(self, msg):
        self.vslam_status = msg.vo_state


    def vslam_check(self):
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

        # This redundancy might be overkill. Maybe there was a past reason...
        delta_time = (current_odom_time - previous_odom_time).nanoseconds * 1e-9
        if delta_time <= 0: return False

        delta_angle = self.min_quat_theta(vslam_odom_msg.pose.pose.orientation, 
                                          self.last_vslam_odom_msg.pose.pose.orientation)
        
        delta_position = self.calculate_3d_displacement(vslam_odom_msg.pose.pose, 
                                                        self.last_vslam_odom_msg.pose.pose)

        # Minor umotion short-circuit efficiency
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
                self.get_logger().info("<<<<< PX4/VSLAM BUFFER DESYNC >>>>>")
                return True

            angle = self.min_quat_theta(drone_odom_msg.pose.pose.orientation,
                                        vslam_odom_msg.pose.pose.orientation)
            
            displacement = self.calculate_3d_displacement(vslam_odom_msg.pose.pose,
                                                          drone_odom_msg.pose.pose)
            if angle >= self.quat_delta_theta or displacement >= self.displacement_delta:
                self.get_logger().info("<<<<< VSLAM RESET MISALIGNMENT >>>>>")
                return True
            self.new_set_pose_call = False

        return False


    def set_slam_pose(self, init=False):
        self.last_set_pose_time = self.get_clock().now()
        last_drone_odom_msg = self._drone_odom_cache.getLast()

        if not last_drone_odom_msg: return

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

        # Orientation: Converted vehicle quaternion in FLU [x, y, z, w]
        req.pose.orientation.x = last_drone_odom_msg.pose.pose.orientation.x
        req.pose.orientation.y = last_drone_odom_msg.pose.pose.orientation.y
        req.pose.orientation.z = last_drone_odom_msg.pose.pose.orientation.z
        req.pose.orientation.w = last_drone_odom_msg.pose.pose.orientation.w
        
        self.get_logger().info(f">>> VSLAM SET POSE <<<")
        self.vslam_busy = True
        self.new_set_pose_call = True

        future = self.set_slam_pose_client.call_async(req)
        future.add_done_callback(self.service_response_callback)


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
        if self.vslam_check() and not self.fmu_lockout:
            if self.ev_fusion_started is False:
                if self.init_flag:
                    self.set_slam_pose(True)
                    self.init_flag = False
                else:
                    self.publish_vslam_to_px4(vslam_odom_msg)
            elif not self.odom_velocity_gate(vslam_odom_msg):
                if not self.odom_displacement_gate(vslam_odom_msg):
                    self.publish_vslam_to_px4(vslam_odom_msg)
                # elif self.odom_temporal_reset_gate(): # Potentially depreciate 
                elif not self.init_flag:
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
        rclpy.shutdown()

if __name__ == '__main__':
    main()