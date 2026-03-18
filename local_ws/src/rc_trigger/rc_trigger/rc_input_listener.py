#!/usr/bin/env python3
import math
import os
from datetime import datetime
from collections import deque
from statistics import median

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from px4_msgs.msg import InputRc
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv

from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
    QoSHistoryPolicy
)


class RcInputListener(Node):
    def __init__(self):
        super().__init__('rc_input_listener')

        self.STATE_BUFFER_N = 5
        self.Z_BUFFER_N = 5
        self.Z_SIGN = 1.0

        # -------- Parameters --------
        self.declare_parameter('overlap', 0.30)
        self.declare_parameter('distanceToWall', 0.70)
        self.declare_parameter('vfov_deg', 39.0)

        self.declare_parameter('rc_trigger_channel', 7)
        self.declare_parameter('rc_trigger_threshold_us', 2000)

        self.declare_parameter('topic_state', '/px4_state_machine/state')
        self.declare_parameter('topic_pose', '/reactor/drone_pose')
        self.declare_parameter('topic_rc', '/fmu/out/input_rc')
        self.declare_parameter('topic_image', '/scan_cam/image_raw')

        overlap = self.get_parameter('overlap').get_parameter_value().double_value
        d_wall_m = self.get_parameter('distanceToWall').get_parameter_value().double_value
        vfov_deg = self.get_parameter('vfov_deg').get_parameter_value().double_value

        self.rc_trigger_channel = self.get_parameter(
            'rc_trigger_channel'
        ).get_parameter_value().integer_value
        self.rc_trigger_threshold_us = self.get_parameter(
            'rc_trigger_threshold_us'
        ).get_parameter_value().integer_value

        self.topic_state = self.get_parameter('topic_state').get_parameter_value().string_value
        self.topic_pose = self.get_parameter('topic_pose').get_parameter_value().string_value
        self.topic_rc = self.get_parameter('topic_rc').get_parameter_value().string_value
        self.topic_image = self.get_parameter('topic_image').get_parameter_value().string_value

        if not (0.0 < overlap < 1.0):
            raise ValueError("overlap should be btw 0 or 1")

        if self.rc_trigger_channel < 1 or self.rc_trigger_channel > 18:
            raise ValueError("rc_trigger_channel must be in [1..18]")

        footprint_H_m = 2.0 * d_wall_m * math.tan(math.radians(vfov_deg) / 2.0)
        self.Z_STEP_M = (1.0 - overlap) * footprint_H_m

        self.get_logger().info(
            f"VFOV={vfov_deg:.2f}deg | Footprint_H={footprint_H_m:.3f}m | "
            f"overlap={overlap:.2f} -> Z_STEP_M={self.Z_STEP_M:.3f}m"
        )
        self.get_logger().info(
            f"RC trigger: CH{self.rc_trigger_channel} > {self.rc_trigger_threshold_us}us"
        )

        # QoS
        self.qos_be_volatile_10 = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.qos_be_volatile_1 = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.qos_be_transient_1 = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Estado interno
        self.state_hist = deque(maxlen=self.STATE_BUFFER_N)
        self.z_hist = deque(maxlen=self.Z_BUFFER_N)
        self.stable_state = None
        self.capture_active = False
        self.capture_dir = 0
        self.last_photo_z = None
        self.pending_photos = 0
        self.rc_high = False
        self.subscription_image = None

        # ==== Directorios de guardado ====
        script_dir = os.path.dirname(os.path.abspath(__file__))
        base_dir = os.path.join(script_dir, 'data_saved')
        os.makedirs(base_dir, exist_ok=True)

        run_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.run_dir = os.path.join(base_dir, f"run_{run_timestamp}")
        os.makedirs(self.run_dir, exist_ok=True)

        self.cycle_counter = 0
        self.current_cycle_dir = None

        self.bridge = CvBridge()

        # Subs
        self.subscription_px4_state = self.create_subscription(
            String, self.topic_state, self.state_callback, self.qos_be_volatile_10
        )
        self.subscription_pose = self.create_subscription(
            PoseStamped, self.topic_pose, self.pose_callback, self.qos_be_volatile_10
        )
        self.subscription_rc = self.create_subscription(
            InputRc, self.topic_rc, self.rc_callback, self.qos_be_transient_1
        )

        self.get_logger().info(f"Node ready. Saving under: {self.run_dir}")

    # ===== utilidades =====

    def _compute_stable_state(self):
        if len(self.state_hist) < self.STATE_BUFFER_N:
            return None
        s0 = self.state_hist[0]
        if all(s == s0 for s in self.state_hist):
            return s0
        return None

    def _compute_z_est(self):
        if len(self.z_hist) == 0:
            return None
        return float(median(self.z_hist))

    def _ensure_image_subscription(self):
        if self.subscription_image is None:
            self.subscription_image = self.create_subscription(
                Image, self.topic_image, self.image_callback, self.qos_be_volatile_1
            )

    def _stop_image_subscription(self):
        if self.subscription_image is not None:
            self.destroy_subscription(self.subscription_image)
            self.subscription_image = None

    def _open_cycle_dir(self):
        """Crear/actualizar carpeta de ciclo sólo cuando entramos en CYCLE_UP."""
        self.cycle_counter += 1
        cycle_label = f"cycle_{self.cycle_counter}"
        self.current_cycle_dir = os.path.join(self.run_dir, cycle_label)
        os.makedirs(self.current_cycle_dir, exist_ok=True)
        self.get_logger().info(f"Started new cycle dir: {self.current_cycle_dir}")

    def _request_photos(self, n: int, reason: str):
        if n <= 0:
            return
        self._ensure_image_subscription()
        self.pending_photos += int(n)
        self.get_logger().info(f"Photo requested x{n} ({reason}), pending={self.pending_photos}")

    def _request_photo(self, reason: str):
        self._request_photos(1, reason)

    # ===== Callbacks =====

    def state_callback(self, msg: String):
        self.state_hist.append(msg.data)

        new_stable = self._compute_stable_state()
        if new_stable is None or new_stable == self.stable_state:
            return

        prev_state = self.stable_state
        self.stable_state = new_stable
        print(f"data: {self.stable_state}\n---", flush=True)

        if self.stable_state == "CYCLE_UP":
            # Nuevo ciclo: sólo aquí creamos carpeta
            if prev_state != "CYCLE_UP":
                self._open_cycle_dir()

            self.capture_active = True
            new_dir = +1

            if self.capture_dir != new_dir:
                self.capture_dir = new_dir
                self.last_photo_z = None
                self.pending_photos = 0

            z_est = self._compute_z_est()
            if z_est is not None and self.last_photo_z is None:
                self.last_photo_z = z_est
                self._request_photo(f"{self.stable_state}_enter")

        elif self.stable_state == "CYCLE_DOWN":
            # Continúa el mismo ciclo, misma carpeta
            self.capture_active = True
            new_dir = -1

            if self.capture_dir != new_dir:
                self.capture_dir = new_dir
                self.last_photo_z = None
                self.pending_photos = 0

            z_est = self._compute_z_est()
            if z_est is not None and self.last_photo_z is None:
                self.last_photo_z = z_est
                self._request_photo(f"{self.stable_state}_enter")

        else:
            # Cualquier otro estado: salimos de ciclo
            self.capture_active = False
            self.capture_dir = 0
            self.pending_photos = 0
            self.last_photo_z = None
            self._stop_image_subscription()

    def pose_callback(self, msg: PoseStamped):
        z_raw = float(msg.pose.position.z)
        z = self.Z_SIGN * z_raw
        self.z_hist.append(z)

        if not self.capture_active:
            return

        z_est = self._compute_z_est()
        if z_est is None:
            return

        if self.capture_dir == 0:
            return

        if self.last_photo_z is None:
            self.last_photo_z = z_est
            self._request_photo(f"{self.stable_state}_first_z_ref")
            return

        if self.capture_dir > 0:
            if z_est >= self.last_photo_z + self.Z_STEP_M:
                steps = int((z_est - self.last_photo_z) // self.Z_STEP_M)
                if steps > 0:
                    self.last_photo_z += steps * self.Z_STEP_M
                    self._request_photos(steps, f"{self.stable_state}_steps_up")
        else:
            if z_est <= self.last_photo_z - self.Z_STEP_M:
                steps = int((self.last_photo_z - z_est) // self.Z_STEP_M)
                if steps > 0:
                    self.last_photo_z -= steps * self.Z_STEP_M
                    self._request_photos(steps, f"{self.stable_state}_steps_down")

    def rc_callback(self, msg: InputRc):
        ch_index = int(self.rc_trigger_channel) - 1

        if len(msg.values) <= ch_index:
            self.get_logger().warn("error reading channels.")
            return

        ch_pwm = msg.values[ch_index]
        high = ch_pwm > int(self.rc_trigger_threshold_us)

        if high and not self.rc_high:
            self._request_photo(f"RC_ch{self.rc_trigger_channel}_rising_edge")

        self.rc_high = high

    def image_callback(self, msg: Image):
        if self.pending_photos <= 0:
            if not self.capture_active:
                self._stop_image_subscription()
            return

        try:
            target_dir = self.current_cycle_dir if self.current_cycle_dir else self.run_dir

            now_str = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            save_path = os.path.join(target_dir, f"cam_front_{now_str}.jpg")

            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            cv.imwrite(save_path, cv_img)

            self.pending_photos -= 1
            self.get_logger().info(f"Saved image: {save_path} (pending={self.pending_photos})")
        except CvBridgeError as e:
            self.get_logger().error(f"Error converting image: {e}")

        if (not self.capture_active) and self.pending_photos <= 0:
            self._stop_image_subscription()


def main(args=None):
    rclpy.init(args=args)
    node = RcInputListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
