import sys

from a.srv import FullNameSumService
import rclpy
from rclpy.node import Node

class MyClient(Node):

    def __init__(self):
        super().__init__('client_name')
        self.cli = self.create_client(FullNameSumService, 'summ_full_name')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = FullNameSumService.Request()

    def send_request(self, last_name, name, first_name):
        self.req.last_name = last_name
        self.req.name = name
        self.req.first_name = first_name
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()
    
    client = MyClient()
    response = client.send_request(str(sys.argv[1]), str(sys.argv[2]), str(sys.argv[3]))
    client.get_logger().info(
        'Result of summ_full_name: for %s + %s + %s = %s' %
        (str(sys.argv[1]), str(sys.argv[2]), str(sys.argv[3]), response.full_name))

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
