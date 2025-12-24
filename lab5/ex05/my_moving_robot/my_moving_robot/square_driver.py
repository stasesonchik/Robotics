import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class SquareDriver(Node):
    def __init__(self):
        super().__init__('square_driver')
        self.pub = self.create_publisher(Twist, '/robot/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.state = "forward"   
        self.state_time = 0.0

        self.forward_duration = 6  
        self.turn_duration = 5.5     

    def timer_callback(self):
        dt = 0.1
        self.state_time += dt
        msg = Twist()

        if self.state == "forward":
            msg.linear.x = 0.5        
            msg.angular.z = 0.0

            if self.state_time >= self.forward_duration:
                self.state = "turn"
                self.state_time = 0.0

        elif self.state == "turn":
            msg.linear.x = 0.0
            msg.angular.z = 0.8        

            if self.state_time >= self.turn_duration:
                self.state = "forward"
                self.state_time = 0.0

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SquareDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
