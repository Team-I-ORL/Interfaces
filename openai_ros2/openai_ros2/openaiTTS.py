import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pathlib import Path
from openai import OpenAI
from playsound import playsound
from ament_index_python.packages import get_package_share_directory
import os
from pathlib import Path


class TTSNode(Node):

    def __init__(self):
        super().__init__('RobotSpeaker')
        self.subscription = self.create_subscription(
            String,
            'speak',
            self.listener_callback,
            1)
        self.subscription  # prevent unused variable warning
        api_key = "sk-proj-x6csxFwwUmUH0S5HMn4BT3BlbkFJw4Kqk431ipnpczNRRxuZ"
        self.TTSclient = OpenAI(api_key=api_key)
        self.processingLock = False
        self.packagePath = get_package_share_directory('openai_ros2')
        # print(self.packagePath)


    def listener_callback(self, msg):
        if self.processingLock:
            self.get_logger().info('Still processing previous message')
            return
        self.processingLock = True
        msg = msg.data
        self.get_logger().info('Heard: "%s"' % msg)
        fileName = msg.replace(" ", "_") + ".mp3"

        filePath = self.packagePath + "/audio/" + fileName
        print(filePath)
        # Check if file already exists
        if not os.path.isfile(filePath):
            # If the file does not exist, generate it
            self.get_logger().info("Generating audio file:" + fileName)
            response = self.TTSclient.audio.speech.create(model='tts-1',voice="alloy",input=msg)
            response.stream_to_file(filePath)
        else:
            self.get_logger().info('Audio file already exists: "%s"' % fileName)
        
        playsound(filePath)
        self.processingLock = False



def main(args=None):
    rclpy.init(args=args)

    ttsnode = TTSNode()

    rclpy.spin(ttsnode)
    ttsnode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()