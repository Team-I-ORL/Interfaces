**NodeMCU ESP8266 and Teensy v3.1**

These are PlatformIO projects for NodeMCU and Teensy microcontroller.

ESP is subscribed to a MQTT broker, which receives MQTT messages via a ROS2 node. From the MQTT commands, the ESP transmits UART signals to Teensy which rotates a motor exactly one revolution.

On host machine (Xavier or Fetch itself) with Docker installed, set up the MQTT broker and ROS2 to control the vending machine wirelessly.


For MQTT Docker broker, run the following command after pulling eclipse-mosquitto.
`docker run -it -p 1883:1883 -v ~/Documents/GIT\ Projects/ros2-humble/mosquitto/config:/mosquitto/config --name mqtt-broker eclipse-mosquitto`

Within ROS2 Container, run the following code to initialize connection to the MQTT broker container. Note that broker host IP address may have to be hardcoded in 'config/params.ros2.yaml'.
`ros2 launch mqtt_client standalone.launch.ros2.xml`

Make a publisher node or simply use `ros2 topic` command to check connection.
`"ros2 topic pub -1 /ping/ros std_msgs/msg/String "{data: \"Hello MQTT\"}"" `

