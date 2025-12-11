import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from concurrent.futures import Future

import sys
from math import pi, sqrt, atan2


class MoveToGoal(Node):

    def __init__(self, x, y, theta):
        super().__init__("move_to_goal")

        self.cmd_vel_pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.pose_sub = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        self.current_pose = Pose()
        self.goal_pose = Pose(x = float(x), y = float(y), theta = float(theta) * pi / 180)

        self.future = Future()
        self.timer = self.create_timer(1, self.move_to_goal)

    def pose_callback(self, pose):
        # Обновить позицию
        self.current_pose = pose

    def send_turtle_msg(self, x_speed, angle_speed):
        tw = Twist()
        tw.linear.x = float(x_speed)
        tw.angular.z = float(angle_speed)

        self.cmd_vel_pub.publish(tw)

    def move_to_goal(self):
        x_d = self.goal_pose.x - self.current_pose.x
        y_d = self.goal_pose.y - self.current_pose.y
        alpha = atan2(y_d, x_d)

        dst_target = sqrt(x_d**2 + y_d**2)
        # Пройдем только часть расстояния, на деле она будет еще меньше
        dst_target *= 0.4

        angle_target = alpha - self.current_pose.theta

        self.get_logger().info(f"Distance left: {dst_target:.4f}")

        # Погрешность измерения расстояния
        eps = 0.15
        if abs(dst_target) > eps:
            self.send_turtle_msg(dst_target, angle_target)
            return

        self.timer.destroy()

        angle_target = self.goal_pose.theta - self.current_pose.theta
        
        self.send_turtle_msg(0.0, angle_target)
        self.get_logger().info(f"Goal Reached")
        self.future.set_result(None)

def main():
    rclpy.init()
    mvg = MoveToGoal(sys.argv[1], sys.argv[2], sys.argv[3])
    rclpy.spin_until_future_complete(mvg, mvg.future) 
    mvg.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
