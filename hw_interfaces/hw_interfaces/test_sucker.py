import rclpy
import time
from rclpy.node import Node
from orbiter_bt.srv import Suck

class SuctionClient(Node):
    def __init__(self):
        super().__init__('suction_client')
        self.client = self.create_client(Suck, '/suction_command')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for suction_command service...')
        self.request = Suck.Request()
        
    def send_request(self, command):
        self.request.command = command
        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        if self.future.result() is not None:
            self.get_logger().info(f'Success: {self.future.result()}')
        else:
            self.get_logger().error('Service call failed')

def main(args=None):
    rclpy.init(args=args)
    node = SuctionClient()
    
    command = True  # Set True to start suction, False to stop it
    node.send_request(command)

    time.sleep(10)

    command = False  # Set True to start suction, False to stop it
    node.send_request(command)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
