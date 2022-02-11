# roboclaw_ros
This is a ROS2 node for the Roboclaw motor controllers made by [Basicmicro Motion Control](https://www.basicmicro.com). This code is a fork of of the [ROS1 code](https://github.com/sonyccd/roboclaw_ros) created by Brad Bazemore.

This work has been llightly tested and known to work with at least a recent Roboclaw 2x7A Motor Controller. 

## Before you begin
Before you use this package you need to tune the controller on the Roboclaw.  This will requare the
installation of the free software [BasicMicro Motion Studio](https://downloads.basicmicro.com/software/BMStudio/setup.exe) (Windows only, make sure you are using the newer BasicMicro version and not the older Ion Motion Studio -- I've seen both downloadable on their website).

You do not need to tune for position just velocity.

Follow [this guide](https://resources.basicmicro.com/auto-tuning-with-motion-studio/) to learn how to tune the controller.  


## Usage

TODO: Discuss how to clone this into your workspace

Simple launch:
ros2 launch roboclaw_ros roboclaw.launch.py

Launch with debug logging:
ros2 launch roboclaw_ros roboclaw.launch.py log_level:=debug

## Usage (OLD)
Just clone the repo into your catkin workspace. It contains the ROS package and the motor controller driver.  Remmeber to make sure ROS has permisions to use the dev port you give it.
```bash
cd <workspace>/src
git clone https://github.com/sonyccd/roboclaw_ros.git
cd <workspace>
catkin_make
source devel/setup.bash
roslaunch roboclaw_node roboclaw.launch



```

## Parameters
The launch file can be configure at the command line with arguments, by changing the value in the launch file or through the rosparam server.

|Parameter|Default|Definition|
|-----|----------|-------|
|dev|/dev/ttyACM0|Dev that is the Roboclaw|
|baud|115200|Baud rate the Roboclaw is configured for|
|address|128|The address the Roboclaw is set to, 128 is 0x80|
|max_speed|2.0|Max speed allowed for motors in meters per second|
|ticks_per_meter|4342.2|The number of encoder ticks per meter of movement|
|base_width|0.315|Width from one wheel edge to another in meters|

## Topics
###Subscribed
/cmd_vel [(geometry_msgs/Twist)](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)  
Velocity commands for the mobile base.
###Published
/odom [(nav_msgs/Odometry)](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html)  
Odometry output from the mobile base.

#IF SOMETHING IS BROEKN:
Please file an issue, it makes it far easier to keep track of what needs to be fixed. It also allows others that might have solved the problem to contribute.  If you are confused feel free to email me, I might have overlooked something in my readme.
