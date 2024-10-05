from jodellSdk.jodellSdkDemo import EvsClawControl_01
from rclpy.node import Node
import rclpy
from orbiter_bt.srv import Suck
from std_msgs.msg import Bool

class Sucker(Node):
    def __init__(self, serial_port='/dev/ttyUSB0', id=9):
        super().__init__('sucker')
        self.sucker = EvsClawControl_01()
        self.id = id
        result = self.sucker.serialOperation(serial_port, 115200, True)
        if result != 1:
            self.get_logger().error('Failed to connect to sucker.')
            print("Availabe Ports: ", self.sucker.searchCom())
            raise RuntimeError('Failed to connect to sucker.')
        self.get_logger().info('Connected to sucker.')

        self.srv = self.create_service(Suck, '/suction_command', self.command_handler)
        self.suc_status_pub = self.create_publisher(Bool, '/suction_status', 10)

        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        vacuum_degree = self.get_vacuum_degree()
        msg = Bool()
        # If vacuum_degree is less than -5, then the suction is engaged, so publish True
        msg.data = vacuum_degree[0] < -5
        self.suc_status_pub.publish(msg)
    
    def command_handler(self, request, response):
        result = False
        if request.command == True:
            result = self.start_sucker()
        else:
            result = self.stop_sucker()

        if result == False:
            self.get_logger().error('Failed to execute command.')
        else:
            self.get_logger().info('Command executed successfully.')

        response.success = result
        return response

    def get_vacuum_degree(self):
        return self.sucker.getDeviceCurrentVacuumDegree(9)

    def start_sucker(self):
        self.get_logger().info('Starting sucker.')
        flag = self.sucker.startOrStopDevice(9, 0, 1, False)
        if flag != 1:
            self.get_logger().error('Failed to start sucker.')
            return False
        return True

    def stop_sucker(self):
        self.get_logger().info('Stopping sucker.')
        flag = self.sucker.startOrStopDevice(9, 0, 0, True)
        if flag != 1:
            self.get_logger().error('Failed to stop sucker.')
            return False
        return True

def main(args=None):
    rclpy.init(args=args)
    sucker = Sucker()
    rclpy.spin(sucker)
    sucker.stop_sucker()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
