import rclpy
import math
import time
from rclpy.action import ActionServer
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen
from turtlesim.msg import Pose

def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


from action_cleaning_robot_interface.action import CleaningTask

class CleaningActionServer(Node):
    def __init__(self):
        super().__init__('cleaning_action_task')
        self.action_server = ActionServer(self, CleaningTask, 'cleaning_task',
                                          self.execute_callback)
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.cli_pen = self.create_client(SetPen, '/turtle1/set_pen')
        while not self.cli_pen.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /turtle1/set_pen service...')
        pose = self.get_current_pose()
        self.x_begin = pose.x
        self.y_begin = pose.y

    
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        task_type = goal_handle.request.task_type

        if task_type == 'clean_circle':
            self.get_logger().info('cleaning area...')
            max_radius = goal_handle.request.area_size
            result = self.clean_circle(max_radius, goal_handle)
            goal_handle.succeed()
            return result

        elif task_type == 'return_home':
            self.get_logger().info('Returning home...')
            result = self.return_home(goal_handle.request.target_x, goal_handle.request.target_y, goal_handle) 
            goal_handle.succeed()
            return result

        else:
            self.get_logger().error(f'Unknown task type: {task_type}')
            goal_handle.abort()
            result = CleaningTask.Result()
            result.success = False
            return result



    def clean_circle(self, max_radius, goal_handler):
        self.set_pen()
        twist_msg = Twist()
        feedback_msg = CleaningTask.Feedback()
        result = CleaningTask.Result()
        feedback_msg.current_cleaned_points = 0
        angular_speed = 1.0
        current_radius = 0.1
        spiral_step = 0.003
        grid_step = 0.2
        total_area = math.pi*max_radius*max_radius
        self.cleaned_points = set()

        while current_radius < max_radius:
            twist_msg.linear.x = current_radius * angular_speed
            twist_msg.angular.z = angular_speed
            self.publisher.publish(twist_msg)
            rclpy.spin_once(self, timeout_sec=0.05)

            pose = self.get_current_pose()
            current_area = math.pi*current_radius*current_radius

            feedback_msg.current_x = pose.x
            feedback_msg.current_y = pose.y
            feedback_msg.progress_percent = int(current_area/total_area*100)
            feedback_msg.current_cleaned_points +=1

            x_cell = round(pose.x / grid_step)
            y_cell = round(pose.y / grid_step)
            self.cleaned_points.add((x_cell, y_cell))
            self.get_logger().info('Feedback: {0}'.format(feedback_msg))
            goal_handler.publish_feedback(feedback_msg)

            current_radius += spiral_step

        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.publisher.publish(twist_msg)
        result.success = True
        result.cleaned_points = len(self.cleaned_points)
        result.total_distance = total_area
        return result

    def return_home(self, x, y, goal_handler):
        self.set_pen(off = 1)
        pose = self.get_current_pose()
        x_cur = pose.x
        y_cur = pose.y
        angle_to_goal = math.atan2(y - y_cur, x - x_cur)
        distance = math.sqrt((x - x_cur)**2 + (y - y_cur)**2)
        feedback_msg = CleaningTask.Feedback()
        result = CleaningTask.Result()
        feedback_msg.current_cleaned_points = 0
        while True:
            pose = self.get_current_pose()
            angle_to_goal = math.atan2(y - pose.y, x - pose.x)
            angle_diff = normalize_angle(angle_to_goal - pose.theta)
            cur_distance = math.sqrt((x - pose.x)**2 + (y - pose.y)**2)
            twist_msg = Twist()

            if abs(angle_diff) > 0.1:
                twist_msg.angular.z = 0.5 if angle_diff > 0 else -0.5
                twist_msg.linear.x = 0.0
            elif cur_distance > 0.1:
                twist_msg.angular.z = 0.0
                twist_msg.linear.x = 0.5
            else:
                twist_msg.angular.z = 0.0
                twist_msg.linear.x = 0.0
                self.publisher.publish(twist_msg)
                break

            feedback_msg.current_x = pose.x
            feedback_msg.current_y = pose.y
            feedback_msg.progress_percent = 100 - int(cur_distance/distance*100)
            print(cur_distance, distance)
            self.get_logger().info('Feedback: {0}'.format(feedback_msg))
            goal_handler.publish_feedback(feedback_msg)
            self.publisher.publish(twist_msg)

        result.success = True
        result.cleaned_points = 0
        result.total_distance = distance
        return result

    def set_pen(self, r = 0, g= 0, b= 0, width = 4, off=0):
        req = SetPen.Request()
        req.r = r
        req.g = g
        req.b = b
        req.width = width
        req.off = off
        self.cli_pen.call_async(req)

    def get_current_pose(self):
        self.future = rclpy.task.Future()
        sub = self.create_subscription(Pose, 'turtle1/pose',
                                        lambda msg: self.future.set_result(msg), 1)
        rclpy.spin_until_future_complete(self, self.future)
        self.destroy_subscription(sub)

        return self.future.result()




def main(args=None):
    rclpy.init(args=args)

    cleaning_server = CleaningActionServer()

    rclpy.spin(cleaning_server)


if __name__ == '__main__':
    main()
