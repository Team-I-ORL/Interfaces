# orbiter_bt package

## Installation

The same as generic ROS2 packages, use `colcon build`, then `source install/setup.sh`

Launch behavior tree by `ros2 run orbiter_bt orbiter_bt_node`

## File Structure

| Folder Name    | Explanation                                          |
| -------------- | ---------------------------------------------------- |
| bt_xml         | Defines behavior tree xmls                           |
| dummy_database | Contains a simple fake yaml file for testing purpose |
| include        | C++ include file                                     |
| launch         | Contains behavior tree launching command             |
| src            | C++ sources                                          |
| srv            | ROS2 service message definition files                |

## Source Codes

| Files                     | Explanation                                                  |
| ------------------------- | ------------------------------------------------------------ |
| dummy_ims_server.cpp      | Defines a simple dummy IMS server for testing                |
| orbiter_bt_node.cpp       | Defines behavior tree, binding between xml tree and actual coded actions |
| orbiter_getItemInfo.cpp   | Defines the BT Action of getting information from the IMS through a client node |
| orbiter_nav_behaviors.cpp | Defines BT Action of GoToPose, by binding with NAV2 stack    |

## Behavior Tree Summary

Official tutorial can be found in https://www.behaviortree.dev/docs/3.8/category/tutorial---basics

Note: For this BT, I am using **version 3.8** because NAV2 uses 3.8 as default, to reduce further complications,  dependency issues, etc, I do not intend to use BT 4.X, but that means there are **No** pre-written ROS2 wrappers, but it is not hard to do it yourself.

### Inputs & outputs in human language

BT gets inputs and outputs from to actions / conditions through publishing to a *Blackboard*, which is essentially a dictionary (key - value pair). Giving output is to publish to the dictionary with a key, and receiving input is to index a key in the dictionary.

For example, in the behavior of getting item location from the IMS, and then navigate to the location,the bahavior is defined as
```
<Sequence>
    <Action ID="getItemInfo" itemInfo="" itemName="default" locOut="{location}"/>
    <Action ID="GoToPose" locIn="{location}"/>
</Sequence>
```

It means, for the Action `getItemInfo`, I will have ports named `itemInfo`, `itemName`, and `locOut`. Among which `locOut` is using key `location` of the blackboard. In code, since `locOut` is settled up as an output port, it will associate a value of `location` to the key and push it to the dictionary.

Similarly, `goToPose` action has port `locIn`. In code, `locIn` is settled up as an input port, thus it will read from the blackboard the value associated with `location`.







 