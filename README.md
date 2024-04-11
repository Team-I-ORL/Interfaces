# Interfaces
Interfaces repo - All interfaces including APIs, UIs etc go here

## Fetch
1. SSH (pwd: a.i.makerspace1001)
2. export ROS_MASTER_URI=???

## Docker
For the Jetson Xavier with Ubuntu 18.04 installed, Docker containers have to be used in order to use ROS2 (Humble). The Dockerfile simply builds the Docker image for ROS2 with ros1_bridge packaged installed.

To use the ros1_bridge to network ROS1 and ROS2, the follow the steps below.

1. Start roscore for ROS1 Melodic (natively installed on Xavier
```
   source /opt/ros/melodic/setup.bash
   roscore
```
2. Build ROS2 Docker Container (if not already built)
```
   cd ros-humble
   sudo docker build -t ros-humble .
```
3. Run ROS2 Docker
```
   sudo docker run -it --name ros-humble --net=host ros-humble
```
4. Source ros and ros1_bridge workspace
```
   source /opt/ros/humble/setup.bash
   source ros-humble-ros1-bridge/install/local_setup.bash
```
5. Start ros1_bridge node in ROS2 docker
   Dynamic bridge to bridge all topics
   ```
   ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
   ```
   Parameter bridge to bridge only selected topics and services:
   Load yaml file with mentioned topics and services to rosparam on ros1
   ```
   ### following commands are to be run on the beast######
   cd ros-humble/config
   rosparam load bridge.yaml
   ```
   To run the bridge with the updated params run the following from the ros2 docker 
   ```
   ros2 run ros1_bridge parameter_bridge
   ```  

Now, ROS2 within the container environment should be connected to ROS1.

### Example
1. In a different Jetson terminal, publish or subscribe to a topic.
```
   source /opt/ros/melodic/setup.bash
   rostopic pub /chatter std_msgs/String hello
```
2. In a different ROS2 docker terminal, subscribe or publish to a topic.
```
   source /opt/ros/humble/setup.bash
   ros2 topic echo /chatter
```
Note: the following terminal command can be used to start another Docker terminal.
```
sudo docker exec ros-humble /bin/bash
```
