#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

import numpy as np


class DepthStop(Node):
    def __init__(self):
        super().__init__('depth_stop')

        self.cmd_pub = self.create_publisher(Twist, '/robot/cmd_vel', 10)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.create_subscription(Image, '/depth/image', self.image_callback, qos)

        self.STOP_DIST = 2
        self.GO_SPEED = 0.3
        self.LOG_THROTTLE_DURATION = 1.0

        self.start_time = time.time()
        self.last_wait_log = 0.0
        self.last_go_log = 0.0
        self.last_stop_log = 0.0

        self.get_logger().info('Waiting 3 seconds...')

    def _throttle_ok(self, last_t: float, period: float) -> bool:
        return (time.time() - last_t) >= period

    def image_callback(self, msg: Image):
        now = time.time()
        elapsed = now - self.start_time

        if elapsed < 3.0:
            if self._throttle_ok(self.last_wait_log, 1.0):
                self.get_logger().info(f'Wait: {3.0 - elapsed:.1f} s')
                self.last_wait_log = now
            return

        cmd = Twist()
        cmd.angular.z = 0.0

        if msg.encoding != '32FC1':
            self.get_logger().error(f'Wrong encoding: {msg.encoding}')
            cmd.linear.x = 0.0
            self.cmd_pub.publish(cmd)
            return

        try:
            # Достаём float32 глубины из bytes
            depth = np.frombuffer(msg.data, dtype=np.float32)
            total = msg.width * msg.height
            if depth.size < total:
                self.get_logger().error(f'Bad image data: got {depth.size} floats, expected {total}')
                cmd.linear.x = 0.0
                self.cmd_pub.publish(cmd)
                return

            depth = depth[:total].reshape((msg.height, msg.width))

            # Центральное окно 20x20 (row: h/2-10 .. h/2+9, col: w/2-10 .. w/2+9)
            r0 = max(0, msg.height // 2 - 10)
            r1 = min(msg.height, r0 + 20)
            c0 = max(0, msg.width // 2 - 10)
            c1 = min(msg.width, c0 + 20)

            window = depth[r0:r1, c0:c1]
            valid = np.isfinite(window) & (window > 0.0)

            if np.any(valid):
                min_distance = float(np.min(window[valid]))
            else:
                min_distance = 10.0  # как у тебя в C++ по умолчанию

            if min_distance < self.STOP_DIST:
                cmd.linear.x = 0.0
                if self._throttle_ok(self.last_stop_log, 0.5):
                    self.get_logger().warn(f'Obstacle at {min_distance:.2f} m')
                    self.last_stop_log = now
            else:
                cmd.linear.x = self.GO_SPEED
                if self._throttle_ok(self.last_go_log, self.LOG_THROTTLE_DURATION):
                    self.get_logger().info(f'Go. Depth: {min_distance:.2f} m')
                    self.last_go_log = now

        except Exception as e:
            self.get_logger().error(f'Error: {e}')
            cmd.linear.x = 0.0

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = DepthStop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
