import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class LidarStop(Node):
    def __init__(self):
        super().__init__('lidar_stop')

        self.STOP_DIST = 2.0
        self.GO_SPEED = 0.3

        self.cmd_pub = self.create_publisher(Twist, '/robot/cmd_vel', 10)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.create_subscription(LaserScan, '/robot/scan', self.scan_cb, qos)

        self.start_time = self.get_clock().now()
        self.moving = False
        self.get_logger().info('Waiting 3 seconds...')

    def scan_cb(self, msg: LaserScan):
        if (self.get_clock().now() - self.start_time).nanoseconds < int(3e9):
            return

        min_dist = float('inf')
        valid_cnt = 0

        for r in msg.ranges:
            if (not math.isnan(r) and not math.isinf(r) and
                msg.range_min <= r <= msg.range_max and r > 0.0):
                valid_cnt += 1
                if r < min_dist:
                    min_dist = r

        cmd = Twist()

        if min_dist == float('inf'):
            cmd.linear.x = 0.0
            state = 'NO_RANGES'
            self.moving = False
        elif min_dist < self.STOP_DIST:
            cmd.linear.x = 0.0
            state = 'STOP'
            self.moving = False
        else:
            cmd.linear.x = self.GO_SPEED
            state = 'GO'
            self.moving = True

        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f"state={state} min={min_dist if min_dist != float('inf') else 'inf'} "
            f"valid={valid_cnt}/{len(msg.ranges)} cmd_v={cmd.linear.x:.2f}",
            throttle_duration_sec=0.5
        )  

def main(args=None):
    rclpy.init(args=args)
    node = LidarStop()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
