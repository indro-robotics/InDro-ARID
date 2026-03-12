#!/usr/bin/env python

import time
import math
import rclpy
import asyncio
import threading
import numpy as np
import rclpy.callback_groups
from collections import deque

# Important later for multithreading, too many update callbacks
from threading import Lock
from tf2_ros import TransformStamped
from rclpy.executors import MultiThreadedExecutor

from rclpy.node import Node
from rclpy.clock import Clock
from std_srvs.srv import Trigger, SetBool
from geometry_msgs.msg import Vector3
from std_msgs.msg import String, Float32, Bool

from rclpy.qos import (QoSProfile,
                       QoSReliabilityPolicy,
                       QoSHistoryPolicy,
                       QoSDurabilityPolicy)

from std_msgs.msg import String, Float32
from px4_msgs.msg import ( GotoSetpoint,
                          VehicleStatus, 
                          VehicleCommand, 
                          DistanceSensor, 
                          VehicleCommandAck, 
                          TrajectorySetpoint,
                          OffboardControlMode,
                          VehicleLandDetected,
                          EstimatorStatusFlags,
                          VehicleLocalPosition)

from state_machine_interfaces.srv import (Launch,
                                          Land,
                                          Cycle,
                                          Halt,
                                          Forceland,
                                          FMUreboot, 
                                          Panic)

from isaac_ros_visual_slam_interfaces.srv import Reset
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
from april_targeting_interfaces.msg import ShelfTarget, AmrTarget
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


class Target:
    def __init__(self, fsm):
        self.fsm = fsm 
        self.position = Vector3()
        self.yaw = 0.0

    def stamp(self):
        self.position = Vector3(
            x=self.fsm.local_position.x,
            y=self.fsm.local_position.y,
            z=self.fsm.local_position.z
        )
        self.yaw = float(self.fsm.local_yaw)

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

    def reset(self):
        self.index = 0
        self.filled = 0

class DRONE_FSM(Node):

    def __init__(self): 
        super().__init__('px4_state_control_node')

        ### VSLAM MANAGEMENT #######################################################################
        self.reset_vslam = self.create_client(Reset, '/visual_slam/reset')
        while not self.reset_vslam.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('VSLAM INITIALIZING...')
            # Should have a system service call to start VSLAM node after a certain # tries

        self.reactor_pose_client = self.create_client(Trigger, '/visual_slam/set_reactor_pose')
        while not self.reactor_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('REACTOR INITIALIZING...')

        
        ### CAMERA PIPELINE MANAGEMENT (node_manager) #############################################
        self.camera_pipe_clients = {
            'front_cv_pipe': self.create_client(SetBool, '/node_manager/front_cv_pipe'),
            'down_cv_pipe':  self.create_client(SetBool, '/node_manager/down_cv_pipe'),
        }
        self.stop_all_client = self.create_client(Trigger, '/node_manager/stop_all')

        for name, client in self.camera_pipe_clients.items():
            while not client.wait_for_service(timeout_sec=3.0):
                self.get_logger().info('NODE MANAGER: waiting for %s...' % name)
        while not self.stop_all_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info('NODE MANAGER: waiting for stop_all...')

        # Async loop for non-blocking service calls from within FSM callbacks
        self.camera_loop = asyncio.new_event_loop()
        self.camera_thread = threading.Thread(
            target=self.camera_loop.run_forever,
            daemon=True
        )
        self.camera_thread.start()

        # Serializes pipeline calls — prevents interleaving when stop and start
        # are submitted back-to-back from FSM state transitions.
        self.camera_switch_lock = asyncio.Lock()

        # Ensure clean slate — kill anything left over from a previous run
        self.stop_all_pipelines()


        ### QoS PARAMETERS #########################################################################
        self.qos_fmu = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.qos_vslam = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.qos_transient = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1 
        )

        self.passive_group = MutuallyExclusiveCallbackGroup()
        self.control_group = MutuallyExclusiveCallbackGroup()
        self.state_group = MutuallyExclusiveCallbackGroup()

        ### SERVICES ###############################################################################
        self.launch_srv_ = self.create_service(Launch, 'launch', self.launch_callback, callback_group=self.passive_group)
        self.land_srv_ = self.create_service(Land, 'land', self.land_callback, callback_group=self.passive_group)
        self.cycle_srv_ = self.create_service(Cycle, 'cycle', self.cycle_callback, callback_group=self.passive_group)
        self.halt_srv_ = self.create_service(Halt, 'halt', self.halt_callback, callback_group=self.passive_group)
        self.force_land_srv_ = self.create_service(Forceland, 'force_land', self.force_land_callback, callback_group=self.passive_group)
        self.px4_reboot_srv_ = self.create_service(FMUreboot, 'fmu_reboot', self.px4_reboot_callback, callback_group=self.passive_group)
        self.panic_srv_ = self.create_service(Panic, 'panic', self.panic_callback, callback_group=self.passive_group)
    
        ### SUBSCRIPTIONS ##########################################################################
        self.land_detection_sub = self.create_subscription(VehicleLandDetected,
                                                          '/fmu/out/vehicle_land_detected',
                                                          self.landed_status_callback,
                                                          self.qos_fmu, 
                                                          callback_group=self.passive_group)
        
        self.rangefinder_sub = self.create_subscription(DistanceSensor,
                                                        '/fmu/out/distance_sensor',
                                                        self.rangefinder_callback,
                                                        self.qos_fmu, 
                                                        callback_group=self.passive_group)
        
        self.status_sub = self.create_subscription(VehicleStatus,
                                                   '/fmu/out/vehicle_status',
                                                   self.vehicle_status_callback,
                                                   self.qos_fmu, 
                                                   callback_group=self.passive_group)
        
        self.vehicle_local_position_sub = self.create_subscription(VehicleLocalPosition,
                                                                   '/fmu/out/vehicle_local_position',
                                                                   self.local_position_callback,
                                                                   self.qos_fmu)
        
        self.vslam_status_sub = self.create_subscription(VisualSlamStatus,
                                                                   '/visual_slam/status',
                                                                   self.visual_slam_status_callback,
                                                                   self.qos_vslam, 
                                                                   callback_group=self.passive_group)
        
        self.comm_ack_sub = self.create_subscription(VehicleCommandAck,
                                                     '/fmu/out/vehicle_command_ack',
                                                     self.comm_ack_callback, 
                                                     self.qos_fmu,
                                                     callback_group=self.passive_group)
        
        self.est_status_sub = self.create_subscription(EstimatorStatusFlags,
                                                     '/fmu/out/estimator_status_flags',
                                                     self.est_status_callback, 
                                                     self.qos_fmu, 
                                                     callback_group=self.passive_group)


        ### CAMERA ALIVE SUBSCRIPTIONS (node_manager) #############################################
        # TRANSIENT_LOCAL (latched) — we get current state immediately on subscribe
        # pipeline_active gates FSM transitions; only set True when the requested pipeline is alive
        self.front_cv_alive_sub = self.create_subscription(Bool,
                                                           '/node_manager/front_cv_pipe/alive',
                                                           self._front_cv_alive_cb,
                                                           self.qos_transient,
                                                           callback_group=self.passive_group)

        self.down_cv_alive_sub  = self.create_subscription(Bool,
                                                           '/node_manager/down_cv_pipe/alive',
                                                           self._down_cv_alive_cb,
                                                           self.qos_transient,
                                                           callback_group=self.passive_group)


        ### TARGETING SPECIFIC SUBSCRIPTIONS #######################################################
        self._shelf_target_sub = self.create_subscription(ShelfTarget, 
                                                     '/targeting/shelf', 
                                                     self.targeting_callback,
                                                     self.qos_vslam)
        
        self._amr_target_sub = self.create_subscription(AmrTarget, 
                                                     '/targeting/amr', 
                                                     self.targeting_callback,
                                                     self.qos_vslam)
        

        ### PUBLISHERS #############################################################################
        self.pub_trajectory_setpoint_ = self.create_publisher(TrajectorySetpoint, 
                                                     '/fmu/in/trajectory_setpoint', 
                                                     self.qos_fmu)

        self.pub_vehicle_command_ = self.create_publisher(VehicleCommand, 
                                                          "/fmu/in/vehicle_command", 5)   
                                                                                                  
        self.pub_offboard_mode_ = self.create_publisher(OffboardControlMode, 
                                                        '/fmu/in/offboard_control_mode', 
                                                        self.qos_fmu)
        
        self.pub_waypoint_ = self.create_publisher(GotoSetpoint, 
                                                   '/fmu/in/goto_setpoint', 
                                                   self.qos_fmu)

        self.pub_april_amr_height_ = self.create_publisher(Float32, 
                                                           '/targeting/amr_height',
                                                           self.qos_transient, 
                                                           callback_group=self.passive_group)

        self.pub_april_shelf_dist_ = self.create_publisher(Float32, 
                                                           '/targeting/shelf_dist',
                                                           self.qos_transient, 
                                                           callback_group=self.passive_group)

        self.pub_fmu_lockout = self.create_publisher(Bool,
                                                     '/px4_state_machine/fmu_lockout',
                                                     self.qos_transient,
                                                     callback_group=self.passive_group)

        self.pub_FSM_state_ = self.create_publisher(String,
                                                    '/px4_state_machine/state', 
                                                    self.qos_fmu,
                                                    callback_group=self.passive_group)
        

        # FSM + offboard_heartbeat timer callbacks (period <2Hz)
        self.regulator_timer_period = 0.1 # seconds
        self.state_timer = self.create_timer(self.regulator_timer_period,
                                             self.state_callback_wrapper,
                                             callback_group=self.state_group)

        # Command callback timers (period ~20ms good practise... no more than 50ms)
        self.control_period = 0.02  # seconds
        self.control_timer = self.create_timer(self.control_period,
                                               self.control_callback_wrapper,
                                               callback_group=self.control_group)
        
        self.cv_rate_period = 0.25  # seconds
        self.target_rate_timer = self.create_timer(self.cv_rate_period,
                                                   self.target_rate_check,
                                                   callback_group=self.state_group)

        self.nav_state = VehicleStatus.NAVIGATION_STATE_POSCTL
        self.arm_state = VehicleStatus.ARMING_STATE_DISARMED

        # Launch Control ###########################################################################
        self.takeoff_requested = False
        self.takeoff_accepted = False
        self.takeoff_request_time_ms = 0
        self.takeoff_timeout_ms = 2000

        # State machine vars #######################################################################
        self.FSM_current_state = "IDLE"
        self.FSM_last_state = "IDLE"
        self.state_init_flag = False

        # FSM timing vars
        self.FSM_curr_time = 0
        self.FSM_state_change_time = 0
        self.FSM_last_state_output_time = 0
        self.FSM_state_change_time_delta = 0
        self.FSM_state_output_interval = 750    # ms
        self.settling_time_ms = 2000            # ms
        self.settling_timestamp_ms = None

        # Flight Assertion Flags ###################################################################
        self.assert_launch = False      # SET FROM SERVICE
        self.assert_cycle = False       # SET FROM SERVICE
        self.assert_land = False        # SET FROM SERVICE
        
        self.assert_arm = False
        self.assert_offboard = False
        self.assert_local_setpoint_tracking = False
        self.assert_local_waypoint_tracking = False
        

        # FMU State/Comm Management Flags ##########################################################
        self.flightCheck = False
        self.failsafe = False
        self.landed_status = False
        self.ekf2_ev_online = False
        self.last_sent_command = None
        self.fmu_lockout = False

        # Positioning and Targeting ################################################################
        # Local Pose
        self.local_position = Vector3()
        self.local_velocity = Vector3()
        self.local_yaw = 0.

        # Local Targeting 
        self.target_visible = False
        self.last_detection_time = None         # init
        self.target_times = deque(maxlen=20)   # last 20 timestamps
        self.target_min_hz = 1.0               # visibility threshold
        self.target_window = 3.0               # seconds of history to use
        self.target_min_samples = 3            # require at least 3 hits before trusting rate
        self.detection_timeout = 3.0            # seconds

        # Tune these if drone is moving too sluggishly around targets
        self.H_vel_min = 0.1   # m/s, horizontal velocity feed-forward minimum
        self.V_vel_min = 0.1   # m/s, vertical velocity feed-forward minimum
        self.A_vel_min = 5.0   # deg/s, angular velocity feed-forward minimum

        self.H_vel_prox_scalar = 1.0
        self.V_vel_prox_scalar = 1.0
        self.A_vel_proc_scalar = 1.0

        self.target_radius = 0.10                   # meters
        self.tracking_target_radius = 0.20          # meters
        
        self.target_velocity_limit = 0.10           # m/s, just for target lock determination
        self.tracking_velocity_limit = 0.20         # m/s, just for target lock determination

        self.target_yaw_tolerance = 1.5             # deg.

        self.target_vel_lim = 0.
        self.target_ang_vel_lim =  20.0             # deg/s

        self.on_target = False
        self.on_target_yaw = False
        self.target_locked = False
        self.on_target_velocity = False
        self.target_proximity = 0.0
        self.velocity_mag = 0.0
        self.target_update = False

        self.target_local_position = Vector3()
        self.target_local_yaw = 0.
        
        self.target_launch = Target(self)
        self.target_AMR_track = Target(self)
        self.target_floor = Target(self)
        self.target_ceil = Target(self)
        
        self.AMR_height = 1.0 
        
        self.position_delta = 0.
        self.yaw_delta = 0.
        self.yaw_delta_sgn = 0.

        # Local Loitering
        self.loiter_height = 1.0

        # TAG TARGETING ############################################################################
        self.amr_seek_displacement = 4.0            # m, height to search above current position
        self.amr_ang_tracking_offset = 0.0
        self.amr_tracking_height = 1.5              # m, height above tag to track
        self.shelf_tracking_distance = 1.0          # m, default displacement from shelves to track

        # Flight velocities ########################################################################
        self.scan_vel_lim = 0.20                    # m/s, default velocity in scanning mode
                                                    # (but set via cycle command)

        self.seek_vel_lim = 0.20                    # m/s, default velocity while in AMR_SEEK
                                                    # (hard coded)

        self.amr_return_vel_lim = 0.25              # m/s, default velocity while in AMR_RETURN, 
                                                    # post-cycle (hard coded)

        self.homing_vel_lim = 0.25                  # m/s, default velocity while homing on floor,
                                                    # pre-cycle (hard coded)

        self.approach_land_vel_lim = 0.25           # m/s, default land approach speed (hard coded)

        self.final_land_vel_lim = 5.0               # m/s, due to MPC limits, will never achieve.
                                                    # Because of use of PX4 MPC_CRAWL velocity, this
                                                    # value just needs to be equal or higher to the 
                                                    # firmware param

        self.track_vel_lim = 1.5                    # m/s, default velocity in AMR_LOCK. This will 
                                                    # be dynamic in the near future based on a 
                                                    # MOTION boolean topic from AMR. The lower this 
                                                    #is, the more stable the drone is about the AMR 
                                                    # tag target. The faster it is, the better it 
                                                    # can track a moving AMR target without losing 
                                                    # it due to camera FoV.  

        # Cycle Paramters ##########################################################################
        self.cycle_height = 0.                      # m
        self.cycle_velocity = self.scan_vel_lim     # m/s
        self.cycle_orientation = 0.                 # deg.

        # Rangefinder ##############################################################################
        self.rng_buf_sz = 7
        self.rng_std_dev = 1.0
        self.rng_buf = Stat_Ring_Buffer(self.rng_buf_sz, self.rng_std_dev)
        
        self.range = 0.

        self.min_alert_range = 0.40                 # Floor detection at this height
        self.min_land_range = 0.40                  # Landing sequence at this height

        self.rangefinder_prox_alert = False
        self.rangefinder_prox_land = False
        self.rangefinder_prox_loiter = False

        # Camera Management ########################################################################
        self.active_pipeline = None              # pipeline name currently requested
        self.down_cv_alive  = False              # updated directly by subscription callback
        self.front_cv_alive = False              # updated directly by subscription callback

        # VSLAM management #########################################################################
        self.vslam_status = 0
        self.vslam_reset_completed = False
        self.vslam_reset_success = False

        self.vslam_set = False
        self.reactor_pose_completed = False
        self.reactor_pose_success = False

        self.vslam_good = False
        self.px4_vslam_delta_ms = 0
        self.px4_vslam_delta_max_ms = 2000 
        self.px4_vslam_timestamp = 0

        self.vslam_failsafe_modes = ["ASSERT_TAKEOFF",
                                     "LAUNCHING", 
                                     "PRE_AMR_SEEK",
                                     "START_AMR_SEEK",
                                     "AMR_SEEK",
                                     "AMR_LOCK", 
                                     "CYCLE_SETUP",
                                     "PRE_CYCLE_SEEK_FLOOR",
                                     "START_CYCLE",
                                     "CYCLE_UP",
                                     "CYCLE_DOWN",
                                     "AMR_ALT_RETURN",
                                     "AMR_LAT_RETURN",
                                     "START_LANDING",
                                     "LAND_APPROACH",
                                     "ASSERT_LAND",
                                     "LANDING",
                                     "ASSERT_STOP",
                                     "RECOVERY"]

    # state governance callback wrappers ###########################################################
    def state_callback_wrapper(self):
        self.FSM_callback()
        self.vslam_health_check()
        self.arm_heartbeat()
        self.offboard_heartbeat()  

    # control governance callback wrappers #########################################################
    def control_callback_wrapper(self):
        self.target_proximity_callback()

        if(self.assert_offboard):
            if(self.assert_local_waypoint_tracking):
                self.goto_callback()
            elif (self.assert_local_setpoint_tracking): 
                self.trajectory_setpoint_callback()
            

    #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    # DRONE FINITE STATE MACHINE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
    def FSM_callback(self):
        self.temporal_update()
        self.state_output()
        self.state_pub()

        # RC override — highest priority, checked before any FSM state.
        # When the RC pilot moves the sticks, PX4 exits Offboard and enters POSCTL.
        # Without this early check, states that call waypoint_track() every tick
        # (AMR_SEEK, CYCLE_UP, etc.) would keep publishing setpoints at 50Hz and
        # suppress the POSCTL detection in vehicle_status_callback.
        if (self.nav_state == VehicleStatus.NAVIGATION_STATE_POSCTL and self.assert_offboard):
            self.unset_offboard()
            self.set_FSM_state("RECOVERY")
            return

        match self.FSM_current_state:
            

            case "IDLE":
                if (self.assert_launch and 
                    self.landed_status and 
                    not self.assert_arm):
                    self.set_FSM_state("INITIATE_RESET")


            case "INITIATE_RESET":
                self.pub_tracking_params()
                self.stop_all_pipelines()
                self.set_FSM_state("STOP_CV_CAMERAS")


            case "STOP_CV_CAMERAS":
                self.stop_all_pipelines()
                self.set_FSM_state("REBOOT_FMU")


            case "REBOOT_FMU":
                self.reset_fmu()
                self.set_FSM_state("FMU_REBOOTING")


            case "FMU_REBOOTING":
                if not self.fmu_lockout:
                    self.vslam_reset()
                    self.pub_lockout_flag()
                    self.set_FSM_state("VSLAM_REBOOTING")
                    

            case "VSLAM_REBOOTING":
                if self.vslam_good: 
                    self.set_FSM_state("START_CV_CAMERAS")
                    #self.set_reactor_pose() # do not need to set if vslam_reset() is called
            

            case "START_CV_CAMERAS":
                self.set_pipeline('down_cv_pipe')
                self.set_FSM_state("INITIALIZING")


            case "INITIALIZING":
                if (self.ekf2_ev_online and
                    #self.vslam_set and # do not need to monitor if vslam_reset() is called
                    self.pipeline_active):
                    self.settling_timestamp_ms = self.time_ms()
                    self.set_FSM_state("SETTLING")

            case "SETTLING":
                time_now_ms = self.time_ms()
                if (self.settling_timestamp_ms is not None and
                    (time_now_ms - self.settling_timestamp_ms) >= self.settling_time_ms):
                    self.settling_timestamp_ms = None
                    self.set_FSM_state("ASSERT_TAKEOFF")
                    self.set_pre_launch()


            case "ASSERT_TAKEOFF":
                self.takeoff_mode()
                if (self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF and 
                    self.flightCheck and not self.failsafe):
                    self.assert_arm = True
                    self.target_local_position = Vector3(x=self.local_position.x,
                                                         y=self.local_position.y,
                                                         z=-self.loiter_height)
                    self.set_FSM_state("LAUNCHING")
            

            case "LAUNCHING":
                # Check if it is just on target for redundancy. Sometimes PX4 nav_state !switch.
                if ((self.on_target_velocity and self.on_target) or 
                    self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER):
                    self.waypoint_track()
                    self.set_FSM_state("PRE_AMR_SEEK")
                

            case "PRE_AMR_SEEK":
                self.stop_pipeline('front_cv_pipe')
                self.set_pipeline('down_cv_pipe')
                self.set_FSM_state("START_AMR_SEEK")


            case "START_AMR_SEEK":
                if (self.on_target and self.pipeline_active):
                    
                # This was changed... maybe not properly lat return
                # if (self.on_target_velocity and self.pipeline_active):

                    # This needs to go to absolute heights, as we might get caught in a loop.
                    # Temp fix below.
                    # need to add a z drift estimation filter here if using permanently...

                    # self.waypoint_track(target=Vector3(x=0., y=0., z=-self.amr_seek_displacement),
                    #                     velocity=self.seek_vel_lim)

                    seek_z = -(self.loiter_height + self.AMR_height + self.amr_seek_displacement)
                    self.waypoint_track(target=Vector3(x=self.target_local_position.x,
                                                       y=self.target_local_position.y,
                                                       z=seek_z),
                                        velocity=self.seek_vel_lim,
                                        relative_position=False)
                    self.set_FSM_state("AMR_SEEK")



            case "AMR_SEEK":
                if self.target_visible:
                    self.waypoint_track(velocity = self.track_vel_lim)
                    self.set_FSM_state("AMR_LOCK")
                
                    
            case "AMR_LOCK":
                if not self.target_visible:
                    self.set_FSM_state("PRE_AMR_SEEK")
                elif self.target_locked:
                    if (self.assert_land):
                        self.set_FSM_state("START_LANDING")
                    elif (self.assert_cycle):
                        self.set_FSM_state("CYCLE_SETUP")
                        self.target_AMR_track.stamp()
                else:
                    # Visible but not yet locked — keep converging toward target
                    self.waypoint_track(velocity=self.track_vel_lim)
            

            case "CYCLE_SETUP":
                self.stop_pipeline('down_cv_pipe')
                self.set_pipeline('front_cv_pipe')
                self.waypoint_track(yaw = self.cycle_orientation)
                self.set_FSM_state("PRE_CYCLE_SEEK_FLOOR")
            

            case "PRE_CYCLE_SEEK_FLOOR":
                if (self.target_locked and self.pipeline_active):
                    # Start descent to impossibly low altitute
                    self.waypoint_track(target=Vector3(x=0., y=0., z=100.), 
                                        velocity=self.homing_vel_lim)
                    self.set_FSM_state("START_CYCLE")


            case "START_CYCLE":
                if self.rangefinder_prox_alert:
                    self.target_floor.stamp()
                    self.target_ceil = Target(self.target_floor)
                    self.target_ceil.position.z = float(self.AMR_height -
                                                        self.target_floor.position.z -
                                                        self.cycle_height)
                    self.waypoint_track(target=Vector3(x=0., y=0., z=self.target_ceil.position.z),
                                        velocity = self.cycle_velocity)

                    self.set_FSM_state("CYCLE_UP")


            case "CYCLE_UP":
                if (self.target_locked):
                    self.waypoint_track(target=Vector3(x=0., y=0., z=-self.target_ceil.position.z),
                                        velocity = self.cycle_velocity)
                    self.set_FSM_state("CYCLE_DOWN")


            case "CYCLE_DOWN":
                if (self.target_locked or self.rangefinder_prox_alert):
                    self.assert_cycle = False   
                    self.set_FSM_state("AMR_ALT_RETURN")


            case "AMR_ALT_RETURN":
                amr_return_altitude = Vector3(x=0., 
                                              y=0., 
                                              z=float(self.target_floor.position.z - self.AMR_height/2))
                self.waypoint_track(target=amr_return_altitude,
                                    velocity=self.amr_return_vel_lim)
                self.set_FSM_state("AMR_LAT_RETURN")


            case "AMR_LAT_RETURN":
                if (self.on_target):
                    self.waypoint_track(target=Vector3(x=self.target_floor.position.x,
                                                       y=self.target_floor.position.y,
                                                       z=self.target_floor.position.z - 
                                                         self.AMR_height/2),
                                        velocity=self.amr_return_vel_lim,
                                        relative_position=False)
                    self.set_FSM_state("PRE_AMR_SEEK") 


            case "START_LANDING":
                self.waypoint_track(target=Vector3(x=self.local_position.x,
                                                   y=self.local_position.y,
                                                   z=self.local_position.z + 100.),
                                        velocity=self.approach_land_vel_lim, 
                                        relative_position=False,
                                        relative_orientation=False)
                self.set_FSM_state("LAND_APPROACH")
                    

            case "LAND_APPROACH":
                if (self.rangefinder_prox_land):
                    self.target_local_position.z = self.local_position.z
                    self.set_FSM_state("ASSERT_LAND")


            case "ASSERT_LAND":           
                if (self.target_locked):
                    self.set_FSM_state("LANDING")
                    self.land_mode()         
                    

            case "LANDING":
                if(self.landed_status):
                    self.set_FSM_state("DISARM")
            
            
            case "DISARM":
                self.set_FSM_state("IDLE")
                self.assert_arm = False
                self.assert_all_reset()
            

            # ERROR STATES +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
            case "ASSERT_STOP":
                self.waypoint_track()
                if (self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND):
                    self.set_FSM_state("RECOVERY")

            case "RECOVERY":
                if (self.assert_arm and self.landed_status):
                    self.set_FSM_state("DISARM")

            case "REBOOT":
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 
                                             1.0)
                self.set_FSM_state("IDLE")
            # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
    #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%





    ################################################################################################
    ### STATE MACHINE MANAGEMENT  ##################################################################
    def temporal_update(self):
        self.FSM_curr_time = self.time_ms()
        self.FSM_state_change_time_delta = self.FSM_curr_time - self.FSM_state_change_time
    
    def set_FSM_state(self, new_state: str):
        self.FSM_last_state = str(self.FSM_current_state)
        self.FSM_current_state = new_state
        self.FSM_state_change_time = self.FSM_curr_time
        self.get_logger().info(f"{self.FSM_last_state} -> {self.FSM_current_state}")
    
    def state_pub(self):  
        fsm_state_msg = String()
        fsm_state_msg.data = self.FSM_current_state
        self.pub_FSM_state_.publish(fsm_state_msg)

    def state_output(self):
        if ((self.time_ms() - self.FSM_last_state_output_time) > self.FSM_state_output_interval):
            self.FSM_last_state_output_time = self.FSM_curr_time
            self.get_logger().info(f"----- STATUS -----")
            self.get_logger().info(f"VSLAM: {self.vslam_good}")
            self.get_logger().info(f"FMU EKF2: {self.ekf2_ev_online}")
            self.get_logger().info(f"FLIGHT_CHECK: {self.flightCheck}")
            self.get_logger().info(f"FAILSAFE: {self.failsafe}")
            self.get_logger().info(f"ARM_STATUS: {self.arm_state}")
            self.get_logger().info(f"NAV_STATUS: {self.nav_state}")
            self.get_logger().info(f"FSM_STATE: {self.FSM_current_state}")
            self.get_logger().info(f"TARGET_DELTA: {self.target_proximity} m")
            self.get_logger().info(f"YAW_DELTA: {self.yaw_delta} °")
            self.get_logger().info(f"VELOCITY: {self.velocity_mag} m/s")
            self.get_logger().info(f"ON_TARGET: {self.on_target} | ON_TARGET_VEL: {self.on_target_velocity} | ON_TARGET_YAW: {self.on_target_yaw}")
            self.get_logger().info(f"PIPELINE_ACTIVE: {self.pipeline_active} ({self.active_pipeline})")
            self.get_logger().info(f"WP_TRACKING: {self.assert_local_waypoint_tracking} | SP_TRACKING: {self.assert_local_setpoint_tracking}")
            self.get_logger().info(f"TERRAIN_RNG: {self.range} m")
            if (self.target_locked): self.get_logger().info(f">>> ON TARGET <<<<")
            self.get_logger().info(f"------------------")
            self.get_logger().info("")
    


    # MISC. ########################################################################################
    ################################################################################################
    def vslam_health_check(self):
        self.px4_vslam_delta_ms = self.time_ms() - self.px4_vslam_timestamp

        if (self.vslam_status == 1 and 
            (self.px4_vslam_delta_ms < self.px4_vslam_delta_max_ms)):
            self.vslam_good = True 
        else:
            self.vslam_good = False
            if (self.FSM_current_state in self.vslam_failsafe_modes):
                self.FSM_current_state = "ASSERT_LAND"

    def set_pipeline(self, target_pipe: str):
        """
        Start a specific pipeline without stopping anything else.
        pipeline_active (property) reflects the alive state of the requested pipeline.
        """
        self.active_pipeline = target_pipe

        async def _async_start():
            async with self.camera_switch_lock:
                client = self.camera_pipe_clients.get(target_pipe)
                if client is None:
                    self.get_logger().error('set_pipeline: unknown pipeline: %s' % target_pipe)
                    return
                if not client.service_is_ready():
                    self.get_logger().error('set_pipeline: node_manager/%s not available' % target_pipe)
                    return
                req_on = SetBool.Request()
                req_on.data = True
                try:
                    await client.call_async(req_on)
                    # pipeline_active set True by alive subscription when confirmed running
                except Exception as e:
                    self.get_logger().warn('set_pipeline: failed to enable %s: %s' % (target_pipe, str(e)))

        asyncio.run_coroutine_threadsafe(_async_start(), self.camera_loop)

    def stop_pipeline(self, pipe_name: str):
        """Stop one specific pipeline. Does not affect pipeline_active or other pipelines."""
        async def _async_stop():
            async with self.camera_switch_lock:
                client = self.camera_pipe_clients.get(pipe_name)
                if client is None:
                    self.get_logger().error('stop_pipeline: unknown pipeline: %s' % pipe_name)
                    return
                if not client.service_is_ready():
                    self.get_logger().warn('stop_pipeline: node_manager/%s not available — skipping' % pipe_name)
                    return
                req_off = SetBool.Request()
                req_off.data = False
                try:
                    await client.call_async(req_off)
                except Exception as e:
                    self.get_logger().warn('stop_pipeline: failed to disable %s: %s' % (pipe_name, str(e)))

        asyncio.run_coroutine_threadsafe(_async_stop(), self.camera_loop)

    def stop_all_pipelines(self):
        """Stop every pipeline the node_manager is running."""
        self.active_pipeline = None

        async def _async_stop_all():
            async with self.camera_switch_lock:
                if not self.stop_all_client.service_is_ready():
                    self.get_logger().warn('stop_all_pipelines: node_manager/stop_all not available — skipping')
                    return
                try:
                    await self.stop_all_client.call_async(Trigger.Request())
                except Exception as e:
                    self.get_logger().warn('stop_all_pipelines: %s' % str(e))

        asyncio.run_coroutine_threadsafe(_async_stop_all(), self.camera_loop)

    def _front_cv_alive_cb(self, msg):
        self.front_cv_alive = msg.data

    def _down_cv_alive_cb(self, msg):
        self.down_cv_alive = msg.data

    @property
    def pipeline_active(self):
        if self.active_pipeline == 'down_cv_pipe':
            return self.down_cv_alive
        if self.active_pipeline == 'front_cv_pipe':
            return self.front_cv_alive
        return False

    def launch_callback(self, request, response):
        self.assert_launch = True  
        self.loiter_height = request.loiter_altitude 
        response.success = True
        return response

    def land_callback(self, request, response):
        self.assert_land = True     
        response.success = True
        return response   
    
    def force_land_callback(self, request, response):
        self.set_FSM_state("ASSERT_LAND")   
        response.success = True
        return response 
    
    def halt_callback(self, request, response):
        self.set_FSM_state("ASSERT_STOP")   
        response.success = True
        return response 
    
    def cycle_callback(self, request, response):
        self.assert_cycle = True     
        self.cycle_height = request.shelf_height
        self.cycle_velocity = request.scan_velocity
        self.shelf_tracking_distance = request.shelf_distance
        self.cycle_orientation = request.amr_orientation
        self.pub_tracking_params()
        response.success = True
        return response

    def panic_callback(self, request, response):
        self.set_FSM_state("RECOVERY")  
        self.assert_arm = False  
        response.success = True
        return response
    
    def px4_reboot_callback(self, request, response):
        self.set_FSM_state("REBOOT") 
        response.success = True
        return response 
        
    def target_rate_check(self):
        now = self.get_clock().now().nanoseconds * 1e-9

        if self.last_detection_time is None or (now - self.last_detection_time) > self.detection_timeout:
            self.target_times.clear()
            self.target_visible = False
            return

        cutoff = now - self.target_window
        while self.target_times and self.target_times[0] < cutoff:
            self.target_times.popleft()

        n = len(self.target_times)
        if n < self.target_min_samples:
            self.target_visible = False
            return

        td = self.target_times[-1] - self.target_times[0]
        if td <= 0.0:
            self.target_visible = False
            return

        rate = (n - 1) / td
        self.target_visible = rate >= self.target_min_hz

    ################################################################################################
    ### OFFBOARD CONTROL ###########################################################################
    def setpoint_track(self, target: Vector3 = None, velocity: float = None, 
                             yaw: float = None, relative_position: bool = True, 
                             relative_orientation: bool = True):
        
        self.set_target(target, velocity, yaw, relative_position, relative_orientation) 
        self.target_proximity_callback()

        self.assert_offboard = True
        self.assert_local_setpoint_tracking = True
        self.assert_local_waypoint_tracking = False

    def waypoint_track(self, target: Vector3 = None, velocity: float = None, 
                             yaw: float = None, relative_position: bool = True, 
                             relative_orientation: bool = True):
                       
        self.set_target(target, velocity, yaw, relative_position, relative_orientation)  
        self.target_proximity_callback()

        self.assert_offboard = True
        self.assert_local_setpoint_tracking = False
        self.assert_local_waypoint_tracking = True  # might be an issue in auto_land

    def unset_offboard(self):
        self.assert_offboard = False
        self.assert_local_setpoint_tracking = False
        self.assert_local_waypoint_tracking = False
    


    ################################################################################################
    ### DRONE GENERIC  #############################################################################
    def set_pre_launch(self):
        self.target_launch.stamp()
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN, \
                                     param5=0.0, param6=0.0, param7=0.0)

    def assert_all_reset(self):
        self.assert_launch = False
        self.assert_cycle = False
        self.assert_land = False 
        
        self.assert_arm = False
        self.assert_offboard = False
        self.assert_local_setpoint_tracking = False
        self.assert_local_waypoint_tracking = False
        
        self.flightCheck = False
        self.failsafe = False
        self.landed_status = False
        self.ekf2_ev_online = False
        self.last_sent_command = None
        self.fmu_lockout = True

        self.local_position = Vector3()
        self.local_velocity = Vector3()
        self.local_yaw = 0.
        
        self.on_target = False
        self.on_target_yaw = False
        self.target_locked = False
        self.on_target_velocity = False
        self.target_proximity = 0.0
        self.velocity_mag = 0.0
        self.target_update = False

        self.target_local_position = Vector3()
        self.target_local_yaw = 0.
        
        self.target_launch = Target(self)
        self.target_AMR_track = Target(self)
        self.target_floor = Target(self)
        self.target_ceil = Target(self)
        
        self.AMR_height = 1.0 
        
        self.position_delta = 0.
        self.yaw_delta = 0.
        self.yaw_delta_sgn = 0.

        self.target_vel_lim = 0. 

        self.loiter_height = 1.0
        self.amr_tracking_height = 1.5          # m
        self.shelf_tracking_distance = 1.0      # m
        
        self.cycle_height = 0.
        self.cycle_velocity = self.scan_vel_lim
        self.cycle_orientation = 0.

        self.range = 0.
        self.rangefinder_prox_alert = False
        self.rangefinder_prox_land = False
        self.rangefinder_prox_loiter = False

        self.vslam_status = 0
        self.vslam_reset_completed = False
        self.vslam_reset_success = False

        self.vslam_set = False
        self.reactor_pose_completed = False
        self.reactor_pose_success = False

        self.vslam_good = False
        self.px4_vslam_delta_ms = 0
        self.px4_vslam_timestamp = 0
        self.cmd_ack = False

        self.takeoff_requested = False
        self.takeoff_accepted = False
        self.takeoff_request_time_ms = 0

        self.vslam_reset_attempts = 0

        self.amr_ang_tracking_offset = 0.

        self.target_times.clear()
        self.target_visible = False

    ################################################################################################
    ### LOCAL TARGETING ############################################################################
    def set_target(self, target: Vector3 = None, velocity: float = None, 
                         yaw: float = None, relative_position: bool = True, 
                         relative_orientation: bool = True):
        
        if target is not None:
            if relative_position == True:
                self.target_local_position = Vector3(x=self.local_position.x + target.x,
                                                     y=self.local_position.y + target.y,
                                                     z=self.local_position.z + target.z)
            else:
                self.target_local_position = Vector3(x=target.x, y=target.y, z=target.z)
        else:
            self.target_local_position = Vector3(x=self.local_position.x,
                                                 y=self.local_position.y,
                                                 z=self.local_position.z)
        if yaw is not None:
            raw_yaw = self.local_yaw + yaw if relative_orientation else yaw
        else:
            raw_yaw = self.local_yaw

        self.target_local_yaw = (raw_yaw + 180) % 360 - 180
        

        if velocity is not None: self.target_vel_lim = float(velocity)


    def target_proximity_callback(self):
        self.position_delta = np.array([(self.target_local_position.x - self.local_position.x), \
                                        (self.target_local_position.y - self.local_position.y), \
                                        (self.target_local_position.z - self.local_position.z)])
        
        self.target_proximity = np.linalg.norm(self.position_delta)

        self.velocity_mag = np.linalg.norm(np.array([self.local_velocity.x, \
                                                     self.local_velocity.y, \
                                                     self.local_velocity.z]))
        
        self.yaw_delta_sgn = ((self.target_local_yaw - self.local_yaw + 180) % 360) - 180
        self.yaw_delta = abs(self.yaw_delta_sgn)

        # If you want to query these booleans for targeting, you have four options.
        # In some cases, orientation may not matter so you would just use on_target 
        # and on_target_velocity. Target locked also takes yaw into account.

        
        if self.FSM_current_state != "AMR_LOCK":
            target_radius = self.target_radius
            target_velocity = self.target_velocity_limit
        else:
            target_radius = self.tracking_target_radius
            target_velocity = self.tracking_velocity_limit

        self.on_target = self.target_proximity <= target_radius
        self.on_target_velocity = self.velocity_mag <= target_velocity
        self.on_target_yaw = self.yaw_delta <= self.target_yaw_tolerance
        self.target_locked = self.on_target and self.on_target_velocity and self.on_target_yaw


    def pub_tracking_params(self):      
        msg = Float32()
        msg.data = float(self.amr_tracking_height)
        self.pub_april_amr_height_.publish(msg)
        msg.data = float(self.shelf_tracking_distance)
        self.pub_april_shelf_dist_.publish(msg)


    def pub_lockout_flag(self):                                   
        msg = Bool()     
        msg.data = self.fmu_lockout
        self.pub_fmu_lockout.publish(msg)



    ################################################################################################
    # SHELF/AMR TARGETING ##########################################################################
    def targeting_callback(self, msg):

        # Guard: drop detections from the wrong camera for the current FSM state.
        if isinstance(msg, AmrTarget) and self.FSM_current_state not in {
            "AMR_SEEK", "AMR_LOCK",
            "START_LANDING", "LAND_APPROACH", "ASSERT_LAND",
        }:
            return
        if isinstance(msg, ShelfTarget) and self.FSM_current_state not in {
            "CYCLE_UP", "CYCLE_DOWN",
        }:
            return

        if isinstance(msg, AmrTarget):
            now = self.get_clock().now().nanoseconds * 1e-9
            self.last_detection_time = now
            self.target_times.append(now)

        # Modify targets on-the-fly...
        target_yaw = -msg.target_yaw
        vehicle_heading_stamp = -msg.local_angle_stamp
        delta_yaw = self.local_yaw - vehicle_heading_stamp

        # In-progress... this is to preserve a shelf offset angle for sideways AMR tracking
        # if self.FSM_current_state is "AMR_LOCK":
        #     self.target_local_yaw = (target_yaw - delta_yaw - amr_ang_tracking_offset + 180) % 360 - 180
        # else:
        #     self.target_local_yaw = (target_yaw - delta_yaw + 180) % 360 - 180
        
        self.target_local_yaw = (target_yaw - delta_yaw + 180) % 360 - 180

        target_position = Vector3(x=msg.target_position.x,
                                y=-msg.target_position.y,
                                z=-msg.target_position.z)
        
        vehicle_local_position_stamp = Vector3(x=msg.local_position_stamp.x,
                                            y=-msg.local_position_stamp.y,
                                            z=-msg.local_position_stamp.z)

        delta_pos = Vector3()
        delta_pos.x = self.local_position.x - vehicle_local_position_stamp.x
        delta_pos.y = self.local_position.y - vehicle_local_position_stamp.y
        delta_pos.z = self.local_position.z - vehicle_local_position_stamp.z

        temp_target_local_position = Vector3()
        temp_target_local_position.x = target_position.x - delta_pos.x
        temp_target_local_position.y = target_position.y - delta_pos.y
        temp_target_local_position.z = target_position.z - delta_pos.z

        if self.FSM_current_state == "AMR_LOCK":
            self.target_local_position = temp_target_local_position 
        else:
            z_const_target_local_position = Vector3(x=temp_target_local_position.x,
                                                    y=temp_target_local_position.y,
                                                    z=self.target_local_position.z)
            self.target_local_position = z_const_target_local_position

        # Vestigal debugging...
        # self.get_logger().info(f"CURRENT_YAW: {self.local_yaw}")
        # self.get_logger().info(f"TARGET_YAW: {self.target_local_yaw}")
        # self.get_logger().info(f"CURRENT_POS: {self.local_position}")
        # self.get_logger().info(f"TARGET_POS: {self.target_local_position}")




    ################################################################################################
    ### HEARTBEATS #################################################################################
    def arm_heartbeat(self):    # Sustain ARMED state
        if (self.assert_arm):
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        else:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        
    def offboard_heartbeat(self):   # Sustain OFFBOARD mode
        if (self.assert_offboard):            
            offboard_msg = OffboardControlMode()
            offboard_msg.timestamp = self.time_us()
            offboard_msg.position = self.assert_local_setpoint_tracking
            offboard_msg.velocity = False
            offboard_msg.acceleration = False
            offboard_msg.attitude = False
            offboard_msg.body_rate = False
            offboard_msg.thrust_and_torque = False
            offboard_msg.direct_actuator = False
            self.pub_offboard_mode_.publish(offboard_msg)    



    ################################################################################################
    ### VSLAM MANAGEMENT ###########################################################################
    def vslam_reset(self):
        self.vslam_reset_attempts = 0
        self._call_vslam_reset()

    def _call_vslam_reset(self):
        if self.vslam_reset_attempts >= 3:  # Max retries
            self.get_logger().error("VSLAM reset failed.")
            return
            
        self.vslam_reset_attempts += 1
        future = self.reset_vslam.call_async(Reset.Request())
        future.add_done_callback(self._vslam_reset_done)

    def _vslam_reset_done(self, future):
        try:
            success = future.result().success
            if success:
                self.vslam_reset_completed = True
                self.vslam_reset_success = True
                self.get_logger().info("VSLAM reset successful")
            else:
                self.get_logger().warn("VSLAM reset failed, retrying...")
                self._call_vslam_reset()
        except Exception as e:
            self.get_logger().error(f"Reset exception: {str(e)}")
            self._call_vslam_reset()

    def vslam_reset_callback(self, reset_vslam_call):
        try:
            self.vslam_reset_success = reset_vslam_call.result().success
            self.vslam_reset_completed = True
        except Exception as e:
            self.get_logger().error(f"VSLAM Reset Fail: {e}")
            self.vslam_reset_completed = True
            self.vslam_reset()  # Retry on exception

    def reset_timeout(self):
        self.vslam_reset_completed = True
        self.get_logger().warn("VSLAM Reset timeout occurred")
        self.vslam_reset()  # Retry after timeout

    def visual_slam_status_callback(self, msg):
        self.vslam_status = msg.vo_state
        self.px4_vslam_timestamp = self.time_ms()
    


    ### REACTOR ####################################################################################
    def set_reactor_pose(self):
        self.vslam_set = False
        # Start async service call
        future = self.reactor_pose_client.call_async(Trigger.Request())
        future.add_done_callback(self.reactor_pose_callback)

    def reactor_pose_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.vslam_set = True
                self.get_logger().info("Reactor pose set")
            else:
                self.get_logger().warn("Reactor pose failed")
        except Exception as e:
            self.get_logger().error(f"Pose error: {str(e)}")



    ################################################################################################
    ### TIMESTAMPING ###############################################################################
    def time_us(self):
        timestamp_us = int(Clock().now().nanoseconds / 1000)
        return(timestamp_us)
    
    def time_ms(self):
        timestamp_ms = int(Clock().now().nanoseconds / 1_000_000)
        return(timestamp_ms)



    ################################################################################################
    ### FLIGHT MODES ###############################################################################
    def takeoff_mode(self):
        if (not self.takeoff_requested) or (self.last_sent_command != 
                                            VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF and 
                                            not self.cmd_ack):
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF,
                                         param7=self.loiter_height)
            self.takeoff_requested = True
            self.takeoff_accepted = False
            self.takeoff_request_time_ms = self.time_ms()
            return

        if self.last_sent_command is None and not self.cmd_ack:
            self.takeoff_requested = False



    # Lands the vehicle
    def land_mode(self):
        # This velocity is kind of baloney but it makes me feel better
        self.waypoint_track(target=Vector3(x=0., y=0., z=100.), velocity=self.final_land_vel_lim)
        

    def set_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)  



    ################################################################################################
    ### VEHICLE COMMAND MANAGEMENT #################################################################
    def reset_fmu(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 1.0)
        self.fmu_lockout = True 
        self.pub_lockout_flag() 

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param3=0.0, param4=0.0, \
                                param5=0.0, param6=0.0, param7=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.param3 = param3
        msg.param4 = param4
        msg.param5 = param5
        msg.param6 = param6
        msg.param7 = param7         # altitude value in takeoff command
        msg.command = command       # command ID
        msg.target_system = 1       # system which should execute the command
        msg.target_component = 1    # component which should execute the command
        msg.source_system = 1       # system sending the command
        msg.source_component = 1    # component sending the command
        msg.from_external = True
        msg.timestamp = self.time_us() # time in microseconds
        self.pub_vehicle_command_.publish(msg)
        self.last_sent_command = command
        self.cmd_ack = False


    def comm_ack_callback(self, msg):
        if msg.command == self.last_sent_command:
            if msg.result == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
                self.cmd_ack = True
            self.last_sent_command = None
        else:
            self.get_logger().debug(f"ACK {msg.command} MISSMATCH")


    def trajectory_setpoint_callback(self):
        trajectory_msg = TrajectorySetpoint()
        trajectory_msg.timestamp = self.time_us()
        trajectory_msg.position[0] = self.target_local_position.x
        trajectory_msg.position[1] = self.target_local_position.y
        trajectory_msg.position[2] = self.target_local_position.z
        trajectory_msg.velocity[0] = float('nan')
        trajectory_msg.velocity[1] = float('nan')
        trajectory_msg.velocity[2] = float('nan')

        trajectory_msg.acceleration[0] = float('nan')
        trajectory_msg.acceleration[1] = float('nan')
        trajectory_msg.acceleration[2] = float('nan')
        trajectory_msg.yaw = float(np.deg2rad(self.target_local_yaw))
        trajectory_msg.yawspeed = float('nan')
        self.pub_trajectory_setpoint_.publish(trajectory_msg)


    def goto_callback(self):
        if self.assert_offboard:  
            # Proportional velocity refinement
            H_prox = np.linalg.norm(np.array([abs(self.target_local_position.x - self.local_position.x),
                                              abs(self.target_local_position.y - self.local_position.y)]))
            V_prox = abs(self.target_local_position.z - self.local_position.z)
            A_prox = abs(self.target_local_yaw - self.local_yaw)

            target_A_vel_lim = max(self.A_vel_min, min((self.A_vel_proc_scalar * A_prox), 
                                    self.target_ang_vel_lim))                    # deg/s
            target_H_vel_lim = max(self.H_vel_min, min((self.H_vel_prox_scalar * H_prox), 
                                self.target_vel_lim))                            # m/s
            target_V_vel_lim = max(self.V_vel_min, min((self.V_vel_prox_scalar * V_prox), 
                                self.target_vel_lim))                            # m/s


            goto_msg = GotoSetpoint()

            goto_msg.flag_set_max_horizontal_speed = True
            goto_msg.flag_set_max_vertical_speed = True
            goto_msg.flag_set_max_heading_rate = True

            goto_msg.timestamp = self.time_us()
            goto_msg.position = [self.target_local_position.x,
                                 self.target_local_position.y,
                                 self.target_local_position.z]
            goto_msg.flag_control_heading = True
            goto_msg.heading = float(np.deg2rad(self.target_local_yaw))

            goto_msg.max_horizontal_speed = target_H_vel_lim                
            goto_msg.max_vertical_speed = target_V_vel_lim               
            goto_msg.max_heading_rate = float(np.deg2rad(target_A_vel_lim))     # rad/s conv.

            self.pub_waypoint_.publish(goto_msg)
    

    def vehicle_status_callback(self, msg): # receives and sets vehicle status values
        
        if (msg.nav_state != self.nav_state):
            self.get_logger().info(f"NAV_STATUS: {msg.nav_state}")
        
        if (msg.arming_state != self.arm_state):
            self.get_logger().info(f"ARM STATUS: {msg.arming_state}")

        if (msg.failsafe != self.failsafe):
            self.get_logger().info(f"FAILSAFE: {msg.failsafe}")
        
        if (msg.pre_flight_checks_pass != self.flightCheck):
            self.get_logger().info(f"FLIGHT_CHECK: {msg.pre_flight_checks_pass}")
        
        self.nav_state = msg.nav_state
        self.arm_state = msg.arming_state
        self.failsafe = msg.failsafe
        self.flightCheck = msg.pre_flight_checks_pass

        if (self.nav_state == VehicleStatus.NAVIGATION_STATE_POSCTL and self.assert_offboard):
            self.unset_offboard()
            self.set_FSM_state("RECOVERY")


    def rangefinder_callback(self, msg):
        self.rng_buf.add(msg.current_distance)
        if  self.rng_buf.is_ready():
            self.range = self.rng_buf.average()
            self.rangefinder_prox_alert = self.range <= self.min_alert_range
            self.rangefinder_prox_land = self.range <= self.min_land_range
            self.rangefinder_prox_loiter = self.range <= self.loiter_height

    
    def landed_status_callback(self, msg):
        self.landed_status = msg.landed and msg.ground_contact and msg.maybe_landed


    def est_status_callback(self, msg):
        self.ekf2_ev_online = msg.cs_ev_pos and msg.cs_ev_yaw and msg.cs_ev_hgt
        self.fmu_lockout = not msg.cs_tilt_align


    def local_position_callback(self, msg):
        self.local_position.x = msg.x
        self.local_position.y = msg.y
        self.local_position.z = msg.z
        self.local_velocity.x = msg.vx
        self.local_velocity.y = msg.vy
        self.local_velocity.z = msg.vz
        self.local_yaw = float(np.rad2deg(msg.heading)) 
    


def main(args=None):

    rclpy.init(args=args)
    cypher_cycle = DRONE_FSM()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=3)
    executor.add_node(cypher_cycle)

    try:
        executor.spin()
    except KeyboardInterrupt:
        cypher_cycle.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        cypher_cycle.destroy_node()
        rclpy.shutdown()

        # Only stop/join if active
        if cypher_cycle.camera_loop.is_running():
            cypher_cycle.camera_loop.call_soon_threadsafe(
                cypher_cycle.camera_loop.stop
            )

        if cypher_cycle.camera_thread.is_alive():
            cypher_cycle.camera_thread.join()


if __name__ == '__main__':
    main()