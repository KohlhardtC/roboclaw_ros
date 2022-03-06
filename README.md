# roboclaw_ros
This is a **ROS2** node for the Roboclaw motor controllers made by [Basicmicro Motion Control](https://www.basicmicro.com). This code is a fork of of the [ROS1 code](https://github.com/sonyccd/roboclaw_ros) created by Brad Bazemore.

This work has been llightly tested and known to work with ROS2 Foxy and a recent Roboclaw 2x7A Motor Controller. 

## Before you begin

### Install Dependancies 

sudo apt install ros-foxy-tf-transformations
sudo pip3 install transforms3d

### Tune the RoboClaw
Before you use this package you need to tune the controller on the Roboclaw.  This will requare the
installation of the free software [BasicMicro Motion Studio](https://downloads.basicmicro.com/software/BMStudio/setup.exe) (Windows only, make sure you are using the newer BasicMicro version and not the older Ion Motion Studio -- I've seen both downloadable on their website).

You do not need to tune for position just velocity.

Follow [this guide](https://resources.basicmicro.com/auto-tuning-with-motion-studio/) to learn how to tune the controller.  

### Set the baud and address in Motion Studio

Make sure at this time that you have the baud and address set to the same values you plan to use as shown in the parameters below. 

### Set device

The device in the launch file provided is set to the default of /dev/ttyACM0. If you have your RoboClaw on a different device, be sure to set that!

## Usage

Clone the repo into your workspace. It contains the ROS2 package and the motor controller driver. Remmeber to make sure ROS has permisions to use the dev port you give it. If you are having trouble, you might find it helpful to first test out you can use the motor driver with thesse [python examples(https://downloads.basicmicro.com/code/roboclaw_python.zip).

### Clone and Build

- cd \<workspace\>/src
- clone
  - **ssh:** git clone git@github.com:KohlhardtC/roboclaw_ros.git 
  - **https:** git clone https://github.com/KohlhardtC/roboclaw_ros.git
- cd \<workspace\>
- source
  - **typical Linux setup:** source /opt/ros/foxy/setup.bash
- colcon build --symlink-install

### Run

Make sure RoboClaw has power and run the following in a new terminal window:

- cd \<workspace\>/src
- source . install/setup.bash
- run 
  - **typical:** ros2 launch roboclaw_ros roboclaw.launch.py 
  - **debugging:** ros2 launch roboclaw_ros roboclaw.launch.py log_level:=debug
  - **debugging and using a different device:** ros2 launch roboclaw_ros roboclaw.launch.py log_level:=debug device:=/dev/ttyUSB0

### Test with teleop_twist_keyboard

- source
  - **typical Linux setup:** source /opt/ros/foxy/setup.bash
- ros2 run teleop_twist_keyboard teleop_twist_keyboard 


## Parameters
The launch file can be configure at the command line with arguments, by changing the value in the launch file or through the rosparam server.

|Parameter|Definition|Default|
|-----|-------|------|
|dev|Dev that is the Roboclaw|/dev/ttyACM0|
|baud|Baud rate the Roboclaw is configured for|115200|
|address|The address the Roboclaw is set to, 128 is 0x80|128|
|max_speed|Max speed allowed for motors in meters per second|2|
|ticks_per_meter|The number of encoder ticks per meter of movement|4342.2|
|base_width|Width from one wheel edge to another in meters|0.315|
|topic_odom_out|Topic to use for outputting wheel odometry.|/odom|

## Topics
### Subscribed
- /cmd_vel [(geometry_msgs/Twist)](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)  
Velocity commands for the mobile base.
### Published
- /odom [(nav_msgs/Odometry)](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html)  
Odometry output from the mobile base.

# IF SOMETHING IS BROEKN:
Please file an issue, it makes it far easier to keep track of what needs to be fixed. It also allows others that might have solved the problem to contribute.  If you are confused feel free to email me, I might have overlooked something in my readme.
