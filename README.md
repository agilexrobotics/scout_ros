# ROS Packages for Scout Mobile Robot

## Packages

This repository contains minimal packages to control the scout robot using ROS. 

* scout_bringup: launch and configuration files to start ROS nodes 
* scout_base: a ROS wrapper around [ugv_sdk](https://github.com/agilexrobotics/ugv_sdk) to monitor and control the scout robot
* scout_description: URDF model for the mobile base, a sample urdf (scout_description/sample/scout_v2_nav.xacro) is provided for customized robot with addtional sensors
* scout_msgs: scout related message definitions

### Update the packages for your customized robot

**Additional sensors**

It's likely that you may want to add additional sensors to the scout mobile platform, such as a Lidar for navigation. In such cases, a new ".xacro" file needs to be created to describe the relative pose of the new sensor with respect to the robot base, so that the sensor frame can be reflected in the robot tf tree. 

A [sample](scout_description/sample/scout_v2_nav.xacro) ".xacro" file is present in this repository, in which the base ".xacro" file of an empty scout platform is first included, and then additional links are defined. 

The nodes in this ROS package are made to handle only the control of the scout base and publishing of the status. Additional nodes may need to be created by the user to handle the sensors.

**Alternative odometry calculation**

By default the scout_base package will publish odometry message to topic "/odom". In case you want to use a different approach to calculate the odometry, for example estimating the position together with an IMU, you could rename the default odometry topic to be something else.

```
$ roslaunch scout_bringup scout_base_robot.launch odom_topic_name:="<custom_name>"
```

## Communication interface setup

Please refer to the [README](https://github.com/westonrobot/ugv_sdk_sdk#hardware-interface) of "ugv_sdk" package for setup of communication interfaces.

#### Note on CAN interface on Nvidia Jetson Platforms

Nvidia Jeston TX2/Xavier/XavierNX have CAN controller(s) integrated in the main SOC. If you're using a dev kit, you need to add a CAN transceiver for proper CAN communication. 

## Basic usage of the ROS packages

1. Install dependent libraries

    ```
    $ sudo apt install -y libasio-dev
    $ sudo apt install -y ros-$ROS_DISTRO-teleop-twist-keyboard
    ```

2. Clone the packages into your catkin workspace and compile

    (the following instructions assume your catkin workspace is at: ~/catkin_ws/src)

    ```
    $ cd ~/catkin_ws/src
    $ git clone https://github.com/agilexrobotics/ugv_sdk.git  
    $ git clone https://github.com/agilexrobotics/scout_ros.git
    $ cd ..
    $ catkin_make
    ```
    
3. Setup CAN-To-USB adapter

* Enable gs_usb kernel module(If you have already added this module, you do not need to add it)
    ```
    $ sudo modprobe gs_usb
    ```
    
* first time use scout-ros package
   ```
   $ rosrun scout_bringup setup_can2usb.bash
   ```
   
* if not the first time use scout-ros package(Run this command every time you turn off the power) 
   ```
   $ rosrun scout_bringup bringup_can2usb.bash
   ```
   
* Testing command
    ```
    # receiving data from can0
    $ candump can0
    ```

4. Launch ROS nodes

* Start the base node for scout using the [scout_bringup/launch/scout_robot_base.launch](scout_bringup/launch/scout_robot_base.launch) file. 
    * Launch Examples:
        - Scout robot: `roslaunch scout_bringup scout_robot_base.launch`
        - Scout mini robot: `roslaunch scout_bringup scout_robot_base.launch is_scout_mini:=true`
        - Scout mini omni robot: `roslaunch scout_bringup scout_robot_base.launch is_scout_mini:=true is_scout_omni:=true`
    * The file launches the following nodes:
        - [scout_base/src/scout_base_node](scout_base/src/scout_base_node.cpp) - which connects to the physical robot via the CAN interface or simulates a robot.
        - [robot_state_publisher](http://wiki.ros.org/robot_state_publisher) - which publishes the robot transforms using a URDF.
        - [rviz](http://wiki.ros.org/rviz) - if enabled
    * The launch file has 8 parameters:
        - port_name: specifies the port used to communicate with the robot, default = "can0"
        - simulated_robot: indicates if launching with a simulation, default = "false"
        - odom_topic_name: sets the name of the topic which calculated odometry is published to, defaults = "odom"
        - is_scout_mini: set to true for chassis of type scout_mini, defaults = "false"
        - is_scout_omni: set to true for chassis of type scout_mini_omni, defaults = "false"
        - pub_tf: set to true to publish the odom to base_link transform on /tf, defaults = "true"
        - rviz_on: set to true to launch RVIZ, defaults = "false"
        - rviz_config: set the RVIZ config file to use, defaults = "scout_bringup/rviz/scout.rviz"

* Start the keyboard tele-op node: `roslaunch scout_bringup scout_teleop_keyboard.launch`
    - **SAFETY PRECAUTION**: The default command values of the keyboard teleop node are high, make sure you decrease the speed commands before starting to control the robot with your keyboard! Have your remote controller ready to take over the control whenever necessary. 
