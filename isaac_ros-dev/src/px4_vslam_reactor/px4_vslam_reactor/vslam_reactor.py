#!/usr/bin/env python
import rclpy
import numpy as np
import message_filters
from rclpy.node import Node
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
from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus


class vslam_reactor(Node):
    def __init__(self): 
        super().__init__('vslam_reactor_node')

        # REACTOR CONTROL PARAMETERS
        self.init_flag = True
        self.vslam_status = 0
        self.vslam_busy = False
        self.new_set_pose_call = False                    
        self.ev_fusion_started = False                      # PX4 EV fusion
        self.vslam_stabilization_time = 0.5                 # seconds
        self.last_set_pose_time = self.get_clock().now()        

        self.fmu_local_position = Vector3Stamped()
        self.last_odom_msg = Odometry()
        self.lin_vel_gate = 15                              # m/s
        self.ang_vel_gate = np.pi*5                         # rad/s
        self.vehicle_ts_delta = 20.0                        # ms
        self.sync_cache_sz = 150                            # keep it tight                                         

        self.quat_delta_theta = np.radians(3.0)             # 3 degrees tolerance
        self.displacement_delta = 0.25                      # meters tolerance

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

        ### BROADCASTERS ###########################################################################
        self.px4_tf_broadcaster = TransformBroadcaster(self)

        ### PUBLISHERS #############################################################################
        self.pub_filtered_odom_ = self.create_publisher(Odometry,
                                                        '/visual_slam/filt_slam_odometry',
                                                         qos_profile=self.qos_vslam,
                                                         callback_group=self.vslam_cbg)
        
        self.pub_px4_pose = self.create_publisher(PoseStamped, 
                                                  '/reactor/px4_pose', 
                                                  qos_profile=self.qos_vslam,
                                                  callback_group=self.gen_processing_cbg)
        
        self.pub_base_pose = self.create_publisher(PoseStamped, 
                                                  '/reactor/base_link_pose', 
                                                  qos_profile=self.qos_vslam,
                                                  callback_group=self.gen_processing_cbg)
        
        self.pub_vehicle_posestamped = self.create_publisher(PoseStamped, 
                                                  '/reactor/vehicle_posestamped', 
                                                  qos_profile=self.qos_vslam,
                                                  callback_group=self.gen_processing_cbg)

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

        self._vehicle_attitude_sub = self.create_subscription(VehicleAttitude,
                                                              '/fmu/out/vehicle_attitude',
                                                              self.vehicle_attitude_callback,
                                                              qos_profile=self.qos_fmu,
                                                              callback_group=self.gen_processing_cbg)
        
        self._vehicle_local_position_sub = self.create_subscription(VehicleLocalPosition,
                                                                    '/fmu/out/vehicle_local_position',
                                                                    self.vehicle_position_callback,
                                                                    qos_profile=self.qos_fmu,
                                                                    callback_group=self.gen_processing_cbg)
        
        # COVARIANCES #################################################################
        # self._odom_sub = message_filters.Subscriber(self, Odometry,
        #                                             '/visual_slam/tracking/odometry', 
        #                                             qos_profile=self.qos_vslam,
        #                                             callback_group=self.vslam_cbg)
        
        self._slam_odom_sub = message_filters.Subscriber(self, Odometry,
                                                         '/visual_slam/vis/slam_odometry',
                                                          qos_profile=self.qos_vslam,
                                                          callback_group=self.vslam_cbg)
        
        self._vehicle_posestamped_sub = message_filters.Subscriber(self, PoseStamped,
                                                                '/reactor/vehicle_posestamped',
                                                                qos_profile=self.qos_vslam,
                                                                callback_group=self.gen_processing_cbg)
        
        ### CACHES #################################################################################
        self._vehicle_posestamped_cache = message_filters.Cache(self._vehicle_posestamped_sub,
                                                           cache_size=self.sync_cache_sz)

        ### SERVICES ###############################################################################
        self.set_slam_pose_client = self.create_client(SetSlamPose, 'visual_slam/set_slam_pose')
        
        self.trigger_slam_pose_service = self.create_service(Trigger, 'visual_slam/set_reactor_pose',
                                                             self.set_slam_pose_callback,
                                                             callback_group=self.gen_processing_cbg)
        
        ### CALLBACK TIMERS ########################################################################
        self.tf_timer = self.create_timer(0.01, self.px4_tf_callback, 
                                         callback_group=self.gen_processing_cbg)  # ~60 Hz

        while not self.set_slam_pose_client.wait_for_service():
             pass

        self._slam_odom_sub.registerCallback(self.slam_odom_callback)

        # COVARIANCES #######################################
        # self._odom_sub.registerCallback(self.odom_callback)
    
    # COVARIANCES ######################
    # def odom_callback(self, odom_msg):
    #     self.last_odom_msg = odom_msg

    ### VEHICLE POSESTAMPED CREATION ###############################################################
    def vehicle_position_callback(self, msg):
        self.fmu_local_position = Vector3Stamped()
        ros_time = rclpy.time.Time(nanoseconds=msg.timestamp * 1000)
        self.fmu_local_position.header.stamp = ros_time.to_msg()
        self.fmu_local_position.header.frame_id = "map"
        self.fmu_local_position.vector.x = msg.x
        self.fmu_local_position.vector.y = msg.y
        self.fmu_local_position.vector.z = msg.z

    def vehicle_attitude_callback(self, msg):
        if (self.fmu_local_position.header.stamp.sec == 0 and
        self.fmu_local_position.header.stamp.nanosec == 0):
            return
        
        attitude_ms = msg.timestamp / 1000.0
        position_ros2_ts = self.fmu_local_position.header.stamp
        position_ms = (position_ros2_ts.sec * 1000) + (position_ros2_ts.nanosec / 1_000_000.0)

        if abs(attitude_ms - position_ms) < self.vehicle_ts_delta:
            ros_time = Time(nanoseconds=msg.timestamp * 1000)
            attitude_ros2_ts = ros_time.to_msg()

            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = attitude_ros2_ts
            pose_stamped.header.frame_id = "map"
            pose_stamped.pose.position.x = self.fmu_local_position.vector.x
            pose_stamped.pose.position.y = self.fmu_local_position.vector.y
            pose_stamped.pose.position.z = self.fmu_local_position.vector.z
            pose_stamped.pose.orientation.x = float(msg.q[1])
            pose_stamped.pose.orientation.y = float(msg.q[2])
            pose_stamped.pose.orientation.z = float(msg.q[3])
            pose_stamped.pose.orientation.w = float( msg.q[0])

            self.pub_vehicle_posestamped.publish(pose_stamped)
    ################################################################################################

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

    def list_to_pose(self, position=None, orientation=None) -> Pose:
        p = Pose()
        if position is not None:
            p.position.x, p.position.y, p.position.z = position
        if orientation is not None:
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = orientation
        return p
    
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
        return [float(q_frd[0]), float(-q_frd[1]), float(-q_frd[2]), float(q_frd[3])]
    
    def est_status_callback(self, msg):
        if not self.ev_fusion_started:
            self.ev_fusion_started = msg.cs_ev_pos
            if self.ev_fusion_started:
                self.destroy_subscription(self.est_status_sub)

    def px4_tf_callback(self):
        last_vehicle_posestamped_msg = self._vehicle_posestamped_cache.getLast()

        if last_vehicle_posestamped_msg is None:
            return
        
        q_vehicle_flu, vehicle_pos_flu = self.process_vehicle_posestamped(last_vehicle_posestamped_msg)

        t = TransformStamped()
        t.header.stamp = last_vehicle_posestamped_msg.header.stamp
        t.header.frame_id = 'map'
        t.child_frame_id = 'px4'

        # Set position
        t.transform.translation.x = vehicle_pos_flu[0]
        t.transform.translation.y = vehicle_pos_flu[1]
        t.transform.translation.z = vehicle_pos_flu[2]

        # Set orientation (already in [x,y,z,w] format)
        t.transform.rotation.x = q_vehicle_flu[0]
        t.transform.rotation.y = q_vehicle_flu[1]
        t.transform.rotation.z = q_vehicle_flu[2]
        t.transform.rotation.w = q_vehicle_flu[3]
        self.px4_tf_broadcaster.sendTransform(t)
        
        # PUBLISH POSES
        self.pub_px4_pose.publish(self.transform_to_pose(t))
        try:
            map_base_link = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            self.pub_base_pose.publish(self.transform_to_pose(map_base_link))
        except:
            pass

    def visual_slam_status_callback(self, msg):
        self.vslam_status = msg.vo_state

    def vslam_check(self):
        if self.vslam_status == 1 and not self.vslam_busy:
            return True
        else:
             return False

    def transform_to_pose(self, tf: TransformStamped) -> PoseStamped:
        pose = PoseStamped()
        pose.header = tf.header
        pose.pose.position.x = tf.transform.translation.x
        pose.pose.position.y = tf.transform.translation.y
        pose.pose.position.z = tf.transform.translation.z
        pose.pose.orientation = tf.transform.rotation
        return pose

    def odom_velocity_gate(self, vslam_odom_msg: Odometry) -> bool:

        if (self.last_odom_msg.header.stamp.sec == 0 and 
            self.last_odom_msg.header.stamp.nanosec == 0):
            for field in Odometry.__slots__:
                setattr(self.last_odom_msg, field, getattr(vslam_odom_msg, field))
            return False

        current_odom_time = rclpy.time.Time.from_msg(vslam_odom_msg.header.stamp)
        previous_odom_time = rclpy.time.Time.from_msg(self.last_odom_msg.header.stamp)

        delta_time = (current_odom_time - previous_odom_time).nanoseconds * 1e-9
        if delta_time == 0: return False

        delta_angle = self.min_quat_theta(vslam_odom_msg.pose.pose.orientation, 
                                          self.last_odom_msg.pose.pose.orientation)
        
        delta_position = self.calculate_3d_displacement(vslam_odom_msg.pose.pose, 
                                                        self.last_odom_msg.pose.pose)
        
        linear_velocity = delta_position / delta_time               # meters/second
        angular_velocity = delta_angle / delta_time                 # rad/second

        # self.get_logger().info(f"LINEAR VELOCITY: {linear_velocity}")
        # self.get_logger().info(f"ANGULAR VELOCITY: {angular_velocity}")

        for field in Odometry.__slots__:
            setattr(self.last_odom_msg, field, getattr(vslam_odom_msg, field))

        if linear_velocity >= self.lin_vel_gate or angular_velocity >= self.ang_vel_gate:
            self.get_logger().info(f"<<<<< VSLAM JUMP DETECTED >>>>>")
            return True
        return False
    
    def odom_temporal_reset_gate(self):
        time_delta = (self.get_clock().now() - self.last_set_pose_time).nanoseconds * 1e-9
        if time_delta < self.vslam_stabilization_time and not self.init_flag:
            return True
        return False
       
    def odom_displacement_gate(self, vslam_odom_msg: Odometry) -> bool:
        # If VSLAM is reset, check if it is within bounds
        if self.new_set_pose_call:
            current_odom_time = rclpy.time.Time.from_msg(vslam_odom_msg.header.stamp)
            vehicle_posestamped_msg = self.sync_msg(current_odom_time, self._vehicle_posestamped_cache)

            if vehicle_posestamped_msg is None:
                self.get_logger().info("<<<<< PX4/VSLAM BUFFER DESYNC >>>>>")
                return True
            q_vehicle_flu, vehicle_pos_flu = self.process_vehicle_posestamped(vehicle_posestamped_msg)

            vehicle_pose = self.list_to_pose(position=vehicle_pos_flu, orientation=q_vehicle_flu)
            angle = self.min_quat_theta(vehicle_pose.orientation,
                                        vslam_odom_msg.pose.pose.orientation)
            
            displacement = self.calculate_3d_displacement(vslam_odom_msg.pose.pose,
                                                          vehicle_pose)
            if angle >= self.quat_delta_theta or displacement >= self.displacement_delta:
                return True
            self.new_set_pose_call = False
        return False

    def process_vehicle_posestamped(self, posestamped_msg: PoseStamped):
         # Quaternion Conversions
        q_vehicle_frd = [posestamped_msg.pose.orientation.x,
                         posestamped_msg.pose.orientation.y,
                         posestamped_msg.pose.orientation.z,
                         posestamped_msg.pose.orientation.w]             # [x, y, z, w] in FRD
        
        q_vehicle_flu = self.quat_frd_to_flu(q_vehicle_frd)
        
        # Position Conversion
        pos_local_vehicle_frd = [posestamped_msg.pose.position.x,
                                 posestamped_msg.pose.position.y,
                                 posestamped_msg.pose.position.z]
        
        pos_local_vehicle_flu = self.position_frd_to_flu(pos_local_vehicle_frd)

        return q_vehicle_flu, pos_local_vehicle_flu
    


    def set_slam_pose(self, init=False):
        self.last_set_pose_time = self.get_clock().now()
         
        last_vehicle_posestamped_msg = self._vehicle_posestamped_cache.getLast()

        if not last_vehicle_posestamped_msg:
            return
        
        q_vehicle_flu_last, pos_local_vehicle_flu_last = self.process_vehicle_posestamped(last_vehicle_posestamped_msg)

        req = SetSlamPose.Request()
        # Position: Converted vehicle position in FLU frame
        if init:
            req.pose.position.x = 0.0
            req.pose.position.y = 0.0
            req.pose.position.z = 0.0
        else:
            req.pose.position.x = pos_local_vehicle_flu_last[0]
            req.pose.position.y = pos_local_vehicle_flu_last[1]
            req.pose.position.z = pos_local_vehicle_flu_last[2]
        # Orientation: Converted vehicle quaternion in FLU [x, y, z, w]
        req.pose.orientation.x = q_vehicle_flu_last[0]
        req.pose.orientation.y = q_vehicle_flu_last[1]
        req.pose.orientation.z = q_vehicle_flu_last[2]
        req.pose.orientation.w = q_vehicle_flu_last[3]
        
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
        if not self.odom_temporal_reset_gate():

        # COVARIANCES ########################################################
        #vslam_odom_msg.pose.covariance = self.last_odom_msg.pose.covariance
        #vslam_odom_msg.twist.covariance = self.last_odom_msg.twist.covariance
            
            self.pub_filtered_odom_.publish(vslam_odom_msg)

    def slam_odom_callback(self, vslam_odom_msg):
        if self.vslam_check():
            if self.ev_fusion_started is False:
                if self.init_flag:
                    self.set_slam_pose(True)
                    self.init_flag = False
                else:
                    self.publish_vslam_to_px4(vslam_odom_msg)
            elif not self.odom_velocity_gate(vslam_odom_msg):
                if not self.odom_displacement_gate(vslam_odom_msg):
                    self.publish_vslam_to_px4(vslam_odom_msg)
                elif self.odom_temporal_reset_gate():
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