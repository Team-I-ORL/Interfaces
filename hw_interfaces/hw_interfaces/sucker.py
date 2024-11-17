from jodellSdk.jodellSdkDemo import EvsClawControl_01
from rclpy.node import Node
import rclpy
from orbiter_bt.srv import Suck
from std_msgs.msg import Bool
import os
# ros2 service call /suction_command orbiter_bt/srv/Suck "{command: true}"
class Sucker(Node):
    def __init__(self, serial_port='/dev/ttyUSB0', id=9):
        super().__init__('sucker')

        os.system('sudo chmod 666 /dev/ttyUSB0')

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

        self.timer = self.create_timer(0.2, self.timer_callback)
        self.get_logger().info('Sucker node initialized.')

    def timer_callback(self):
        vacuum_degree = self.get_vacuum_degree()
        # print(vacuum_degree)
        msg = Bool()
        # If vacuum_degree is less than -5, then the suction is engaged, so publish True
        print(vacuum_degree[0])
        msg.data = int(vacuum_degree[0]) < -5
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
#####################################################################################################

# from jodellSdk.jodellSdkDemo import *

# import time
# clawTool = EvsClawControl_01()

# print(clawTool.searchCom())
# # for i in range(10):
# flag = clawTool.serialOperation('/dev/ttyUSB0', 115200, True)
# print(flag)
# print(clawTool.startOrStopDevice(9,0,1,False))

# startTime = time.time()
# while True:
#     if time.time() - startTime > 10:
#         break
#     time.sleep(0.5)
#     data = clawTool.getDeviceCurrentVacuumDegree(9)
#     print("VD: ", data)

# clawTool.startOrStopDevice(9,0,0,True)

