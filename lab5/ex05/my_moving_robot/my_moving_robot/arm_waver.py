import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math

class ArmWaver(Node):
    def __init__(self):
        super().__init__('arm_waver')
        
        self.left_shoulder_pub = self.create_publisher(Float64, '/robot/left_shoulder_cmd', 10)
        self.left_elbow_pub = self.create_publisher(Float64, '/robot/left_elbow_cmd', 10)
        self.right_shoulder_pub = self.create_publisher(Float64, '/robot/right_shoulder_cmd', 10)
        self.right_elbow_pub = self.create_publisher(Float64, '/robot/right_elbow_cmd', 10)
        
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.time = 0.0
        
        self.shoulder_base_angle = math.pi / 2.0
        
    def timer_callback(self):
        self.time += 0.05
        elbow_angle = math.pi / 4.0 * (1.0 + math.sin(2.0 * math.pi * self.time))
        
        shoulder_msg = Float64()
        shoulder_msg.data = self.shoulder_base_angle
        
        elbow_msg = Float64()
        elbow_msg.data = elbow_angle
        
        self.left_shoulder_pub.publish(shoulder_msg)
        self.left_elbow_pub.publish(elbow_msg)
        self.right_shoulder_pub.publish(shoulder_msg)
        self.right_elbow_pub.publish(elbow_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArmWaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()