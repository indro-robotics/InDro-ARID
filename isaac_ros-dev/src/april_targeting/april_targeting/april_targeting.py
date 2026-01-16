#!/usr/bin/env python

# --- Standard ---
import math
from math import pi

# --- Third-party ---
import numpy as np
from scipy.spatial.transform import Rotation as R

# --- ROS2 Core ---
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import (QoSProfile,
                       QoSReliabilityPolicy,
                       QoSHistoryPolicy,
                       QoSDurabilityPolicy)
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

# --- ROS2 Message and TF ---
import message_filters
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3, Point, PoseStamped, Quaternion, TransformStamped

# --- AprilTag Interfaces ---
from apriltag_msgs.msg import AprilTagDetectionArray, AprilTagDetection
from april_targeting_interfaces.msg import ShelfTarget, AmrTarget

# --- TF2 and Transforms ---
from tf2_ros import Buffer, TransformListener, TransformException
import tf_transformations
from tf_transformations import (quaternion_multiply,
                                quaternion_from_euler,
                                quaternion_inverse)



####################################################################################################
# RING BUFFER CLASS ################################################################################
class Stat_Ring_Buffer:
    def __init__(self, size=3, sigma_multiplier=1.5):
        self.size = size
        self.sigma_multiplier = sigma_multiplier
        self.buffer = np.empty(size)
        self.index = 0
        self.filled = 0

    def add(self, value):
        self.buffer[self.index] = value
        self.index = (self.index + 1) % self.size
        self.filled = min(self.filled + 1, self.size)

    def average(self):
        if self.filled < self.size:
            return np.nan
        arr = self.buffer
        mean = np.mean(arr)
        std = np.std(arr)
        lower = mean - self.sigma_multiplier * std
        upper = mean + self.sigma_multiplier * std
        filtered = arr[(arr >= lower) & (arr <= upper)]
        return np.mean(filtered) if filtered.size > 0 else np.nan
    
    def is_ready(self):
        return self.filled >= self.size



class april_tracker(Node):
    def __init__(self): 
        super().__init__('april_tracker_node')


        ############################################################################################
        ### QoS PARAMETERS #########################################################################
        self.qos_fmu = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                  durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                  history=QoSHistoryPolicy.KEEP_LAST,
                                  depth=1)
    
        self.qos_vslam = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                    durability=QoSDurabilityPolicy.VOLATILE,
                                    history=QoSHistoryPolicy.KEEP_LAST,
                                    depth=1)

        self.april_qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                                    durability=QoSDurabilityPolicy.VOLATILE,
                                    history=QoSHistoryPolicy.KEEP_LAST,
                                    depth=1)



        ############################################################################################
        ### PUBLISHERS #############################################################################
        self.pub_shelf_target_ = self.create_publisher(ShelfTarget,
                                                       '/targeting/shelf',
                                                       self.qos_vslam)
        
        self.pub_amr_target_ = self.create_publisher(AmrTarget,
                                                     '/targeting/amr',
                                                     self.qos_vslam)
        
        # Drone target pose (debugging)
        self.pub_drone_3D_target_ = self.create_publisher(PoseStamped, 
                                                        '/targeting/drone_target_pose',
                                                        self.qos_vslam)



        ############################################################################################
        ### SUBSCRIBERS ############################################################################

        self.sub_april_amr_height_ = self.create_subscription(Float32,
                                                              '/targeting/amr_height',
                                                              self.amr_height_callback, 3)
        
        self.sub_april_shelf_dist_ = self.create_subscription(Float32,
                                                              '/targeting/shelf_dist',
                                                              self.shelf_dist_callback, 3)



        ############################################################################################
        # FILTER SUBSCRIBERS #######################################################################
        self._drone_odom_sub = message_filters.Subscriber(self, Odometry,
                                                          '/reactor/drone_odom',
                                                          qos_profile=self.qos_vslam)

        self._shelf_detections = message_filters.Subscriber(self, AprilTagDetectionArray,
                                                            '/cam_front/detections',
                                                            qos_profile=self.april_qos)

        self._amr_detections = message_filters.Subscriber(self, AprilTagDetectionArray,
                                                          '/cam_down/detections',
                                                          qos_profile=self.april_qos)

        self.shelf_tag_sync = message_filters.ApproximateTimeSynchronizer([self._shelf_detections, 
                                                                           self._drone_odom_sub],
                                                                           queue_size=100,
                                                                           slop=0.4)
        
        self.amr_tag_sync = message_filters.ApproximateTimeSynchronizer([self._amr_detections,
                                                                         self._drone_odom_sub],
                                                                         queue_size=100,
                                                                         slop=0.4)

        self.shelf_tag_sync.registerCallback(self.shelf_targeting)
        self.amr_tag_sync.registerCallback(self.amr_targeting)

        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=1.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)



        ############################################################################################
        # VARIABLES ################################################################################
        self.shelf_scan_distance = 1.0          # meters, default
        self.amr_april_z_displacement = 1.25    # meters, default
        self.max_shelf_dist = 3.5               # meters, default

        self.shelf_plane_points = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0]])
        self.shelf_z_vec = np.array([0, 0, -1])
        self.amr_x_vec = np.array([1, 0, 0])



        ############################################################################################
        # STATISTICAL RING FILTERS #################################################################
        self.tag_delta_a_buffer_size = 6   
        self.tag_ang_std_dev = 1.0         

        self.tag_delta_d_buffer_size = 3   
        self.tag_d_std_dev = 1.0        

        self.tag_proj_buffer_size = 3      
        self.tag_proj_std_dev = 1.0        
        
        self.amr_buffer_size = 3
        self.amr_std_dev = 1.5

        self.delta_a_buffer = Stat_Ring_Buffer(self.tag_delta_a_buffer_size, self.tag_ang_std_dev)
        self.delta_d_buffer = Stat_Ring_Buffer(self.tag_delta_d_buffer_size, self.tag_d_std_dev)
        self.proj_x_buffer = Stat_Ring_Buffer(self.tag_proj_buffer_size, self.tag_proj_std_dev)
        self.proj_y_buffer = Stat_Ring_Buffer(self.tag_proj_buffer_size, self.tag_proj_std_dev)
        self.proj_z_buffer = Stat_Ring_Buffer(self.tag_proj_buffer_size, self.tag_proj_std_dev)

        self.amr_map_xy_ang_buffer = Stat_Ring_Buffer(self.amr_buffer_size, self.amr_std_dev)
        self.amr_map_pos_x_buffer = Stat_Ring_Buffer(self.amr_buffer_size, self.amr_std_dev)
        self.amr_map_pos_y_buffer = Stat_Ring_Buffer(self.amr_buffer_size, self.amr_std_dev)
        self.amr_map_pos_z_buffer = Stat_Ring_Buffer(self.amr_buffer_size, self.amr_std_dev)

        # Buffer pointer list
        self.shelf_buffer_list = [self.delta_a_buffer,
                                  self.delta_d_buffer, 
                                  self.proj_x_buffer, 
                                  self.proj_y_buffer,
                                  self.proj_z_buffer]
        
        # Buffer pointer list
        self.amr_buffer_list = [self.amr_map_xy_ang_buffer, 
                                self.amr_map_pos_x_buffer,
                                self.amr_map_pos_y_buffer,
                                self.amr_map_pos_z_buffer]
    


    ################################################################################################
    # CALLBACKS ####################################################################################
    def amr_height_callback(self, msg):
        self.amr_april_z_displacement = msg.data

    def shelf_dist_callback(self, msg):
        self.shelf_scan_distance = msg.data



    ################################################################################################
    # HELPERS ######################################################################################
    def px4_yaw_deg(self, q):
        px4_yaw = math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                      1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
        return px4_yaw


    def shelf_target_projection(self, current_position: Vector3,
                                      norm_proj_vect: Vector3,
                                      displacement: float) -> Vector3:
        return Vector3(x = current_position.x + (norm_proj_vect.x * displacement),
                       y = current_position.y + (norm_proj_vect.y * displacement),
                       z = current_position.z + (norm_proj_vect.z * displacement))


    def proc_amr_TF(self, tf):
        amr_target_pos = Vector3(x=tf.transform.translation.x,
                                 y=tf.transform.translation.y,
                                 z=float(tf.transform.translation.z + self.amr_april_z_displacement))

        q_tf = np.array([tf.transform.rotation.x,
                         tf.transform.rotation.y,
                         tf.transform.rotation.z,
                         tf.transform.rotation.w])

        r = R.from_quat(q_tf)
        amr_target_yaw = r.as_euler('zyx')[0] + np.pi/2

        return amr_target_pos, amr_target_yaw


    def proc_shelf(self, tf: TransformStamped, drone_odom_msg: Odometry):
        
        # Yaw displacement calculations
        shelf_quat_map = [tf.transform.rotation.x,
                          tf.transform.rotation.y,
                          tf.transform.rotation.z,
                          tf.transform.rotation.w]
        
        shelf_z = R.from_quat(shelf_quat_map).apply([0.0, 0.0, 1.0])
        shelf_z_xy = np.array([shelf_z[0], shelf_z[1]])

        if np.linalg.norm(shelf_z_xy) < 1e-6:
            shelf_yaw_map = 0.0
        else:
            shelf_z_angle_xy = np.arctan2(shelf_z_xy[1], shelf_z_xy[0])
            shelf_yaw_map = (shelf_z_angle_xy + 2.0 * np.pi) % (2.0 * np.pi) - np.pi

        # Position displacement calculations
        shelf_pos_map = np.array([tf.transform.translation.x,
                                  tf.transform.translation.y,
                                  tf.transform.translation.z])

        odom_pos_map = np.array([drone_odom_msg.pose.pose.position.x,
                                 drone_odom_msg.pose.pose.position.y,
                                 drone_odom_msg.pose.pose.position.z])

        shelf_to_odom_vect = odom_pos_map - shelf_pos_map
        shelf_z = R.from_quat(shelf_quat_map).apply([0, 0, 1])
        shelf_displacement = np.dot(shelf_to_odom_vect, shelf_z)
        
        # Projection
        closest_pt_on_shelf = odom_pos_map - np.dot(shelf_to_odom_vect, shelf_z) * shelf_z
        vec = odom_pos_map - closest_pt_on_shelf
        vec_xy = np.array([vec[0], vec[1], 0])
        
        if np.linalg.norm(vec_xy) < 1e-6:
            shelf_proj_vect = np.zeros(3)
        else:
             shelf_proj_vect = vec_xy / np.linalg.norm(vec_xy)

        return (shelf_yaw_map, shelf_displacement, shelf_proj_vect)
    
    

    ################################################################################################
    # SHELF TARGETING ##############################################################################
    def shelf_targeting(self, april_detection_msg: AprilTagDetectionArray, drone_odom_msg: Odometry):

        query_time = Time.from_msg(april_detection_msg.header.stamp)

        try:    
            map_shelf_transform = self.tf_buffer.lookup_transform(
                target_frame='map',
                source_frame='shelf',
                time=query_time,
                timeout=Duration(seconds=0.3))

        except Exception as ex:
            self.get_logger().warn(f"Shelf TF Desync: {str(ex)}")
            return

        # Add new values to statistical ring buffers
        shelf_yaw_map, shelf_displacement, shelf_proj_vect_map = self.proc_shelf(map_shelf_transform, drone_odom_msg)

        self.delta_a_buffer.add(shelf_yaw_map)
        self.delta_d_buffer.add(shelf_displacement)
        self.proj_x_buffer.add(shelf_proj_vect_map[0])
        self.proj_y_buffer.add(shelf_proj_vect_map[1])
        self.proj_z_buffer.add(shelf_proj_vect_map[2])

        if drone_odom_msg is not None and all(buf.is_ready() for buf in self.shelf_buffer_list):
            avg_shelf_delta_a = self.delta_a_buffer.average()
            avg_shelf_delta_d = self.delta_d_buffer.average()
            avg_shelf_proj_x = self.proj_x_buffer.average()
            avg_shelf_proj_y = self.proj_y_buffer.average()
            avg_shelf_proj_z = self.proj_z_buffer.average()
            
            if avg_shelf_delta_d < self.max_shelf_dist:
                # Normalize
                magnitude = (avg_shelf_proj_x**2 + avg_shelf_proj_y**2 + avg_shelf_proj_z**2) ** 0.5
                                
                shelf_proj_vect_map_filt = Vector3(x=avg_shelf_proj_x/magnitude,
                                                   y=avg_shelf_proj_y/magnitude,
                                                   z=avg_shelf_proj_z/magnitude)
                
                drone_pos_map = Vector3(x=drone_odom_msg.pose.pose.position.x,
                                        y=drone_odom_msg.pose.pose.position.y,
                                        z=drone_odom_msg.pose.pose.position.z)

                drone_yaw_map = self.px4_yaw_deg(drone_odom_msg.pose.pose.orientation)

                # Positional Targeting
                target_displacement = self.shelf_scan_distance - avg_shelf_delta_d

                drone_shelf_target_map = self.shelf_target_projection(drone_pos_map,
                                                                      shelf_proj_vect_map_filt,
                                                                      target_displacement)

                shelf_msg = ShelfTarget()
                shelf_msg.target_yaw = math.degrees(avg_shelf_delta_a)
                shelf_msg.target_position = drone_shelf_target_map
                shelf_msg.local_angle_stamp = drone_yaw_map
                shelf_msg.local_position_stamp = drone_pos_map

                apriltag_pose_msg = PoseStamped()
                apriltag_pose_msg.header.stamp = map_shelf_transform.header.stamp
                apriltag_pose_msg.header.frame_id = 'map'
                apriltag_pose_msg.pose.position.x = drone_shelf_target_map.x
                apriltag_pose_msg.pose.position.y = drone_shelf_target_map.y
                apriltag_pose_msg.pose.position.z = drone_shelf_target_map.z

                q =quaternion_from_euler(0, 0, avg_shelf_delta_a)
                apriltag_pose_msg.pose.orientation.x = q[0]
                apriltag_pose_msg.pose.orientation.y = q[1]
                apriltag_pose_msg.pose.orientation.z = q[2]
                apriltag_pose_msg.pose.orientation.w = q[3]
                
                self.pub_shelf_target_.publish(shelf_msg)
                self.pub_drone_3D_target_.publish(apriltag_pose_msg)
    


    ################################################################################################
    # AMR TARGETING ################################################################################
    def amr_targeting(self, april_detection_msg: AprilTagDetectionArray, drone_odom_msg: Odometry):
        
        query_time = Time.from_msg(april_detection_msg.header.stamp)

        try:  
            map_amr_transform = self.tf_buffer.lookup_transform(
                target_frame='map',
                source_frame='amr',
                time=query_time,
                timeout=Duration(seconds=0.3))
        
        except Exception as ex:
            self.get_logger().warn(f"AMR TF Desync: {str(ex)}")
            return
        
        amr_target_pos, amr_target_yaw = self.proc_amr_TF(map_amr_transform)
            
        self.amr_map_xy_ang_buffer.add(amr_target_yaw)
        self.amr_map_pos_x_buffer.add(amr_target_pos.x)
        self.amr_map_pos_y_buffer.add(amr_target_pos.y)
        self.amr_map_pos_z_buffer.add(amr_target_pos.z)

        if drone_odom_msg is not None and all(buf.is_ready() for buf in self.amr_buffer_list):
            avg_amr_map_xy_ang = self.amr_map_xy_ang_buffer.average()
            avg_amr_map_pos_x = self.amr_map_pos_x_buffer.average()
            avg_amr_map_pos_y = self.amr_map_pos_y_buffer.average()
            avg_amr_map_pos_z = self.amr_map_pos_z_buffer.average()

            vehicle_local_position = Vector3(x=drone_odom_msg.pose.pose.position.x,
                                             y=drone_odom_msg.pose.pose.position.y,
                                             z=drone_odom_msg.pose.pose.position.z)
            
            vehicle_local_yaw = self.px4_yaw_deg(drone_odom_msg.pose.pose.orientation)

            target_local_yaw_amr = math.degrees(avg_amr_map_xy_ang)

            amr_targeting_msg = AmrTarget()
            amr_targeting_msg.target_yaw = target_local_yaw_amr
            amr_targeting_msg.target_position = amr_target_pos
            amr_targeting_msg.local_angle_stamp = vehicle_local_yaw
            amr_targeting_msg.local_position_stamp = vehicle_local_position
            self.pub_amr_target_.publish(amr_targeting_msg)

            # Drone target pose (debugging)
            drone_3D_target = PoseStamped()
            drone_3D_target.header = map_amr_transform.header
            drone_3D_target.header.frame_id = 'map'
            drone_3D_target.pose.position = Point(x=amr_target_pos.x, 
                                                  y=amr_target_pos.y, 
                                                  z=amr_target_pos.z)
            quat = tf_transformations.quaternion_from_euler(0, 0, avg_amr_map_xy_ang)
            drone_3D_target.pose.orientation = Quaternion(x=quat[0],
                                                          y=quat[1],
                                                          z=quat[2],
                                                          w=quat[3])
            self.pub_drone_3D_target_.publish(drone_3D_target)



def main(args=None):
    rclpy.init(args=args)
    tracker = april_tracker()
    executor = MultiThreadedExecutor(num_threads=2)  # Match the number of timers/callbacks
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