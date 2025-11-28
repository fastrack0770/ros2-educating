# ROS2 education project

[![CI](https://github.com/fastrack0770/ros2-educating/actions/workflows/ci.yml/badge.svg)](https://github.com/fastrack0770/ros2-educating/actions/workflows/ci.yml)

Based on udemy ROS2CRobotics course.  
Contains two packages:  
1. `udemy_ros2_pkg`. This package was made during the course completion. Using only as a reference material.  
2. `wheeled_model_enhanced`. This package was based on the course final work and now is developing further. Contains nodes for a robot that can move around and show what it sees on a camera. The robot simulation is based on gazebo.  

## Requirements
To work with this project, following conditions must be met:
1. OS - `Ubuntu 22.04 LTS`
2. ROS2 version - `Humble`  

## Usage
All commands are provided like they will be executed from the repo root.  

To build the package:  
`source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-up-to wheeled_model_enhanced`  

To import the package:  
`source ./install/setup.bash`  

To run the simulation:  
`ros2 launch wheeled_model_enhanced wheeled_model_enhanced_simulation.launch.py`.  

To turn the robot to the waypoint:  
`ros2 run wheeled_model_enhanced reach_goal_action_client`  

## wheeled_model_enhanced
Contains:
1. wheeled_model_enhanced model and other models  
2. gazebo simulation world where the wheeled_model_enhanced model is placed
2. launch file to run the simulation and nodes
3. reach_goal_action_server node to move the robot around

### Models
All models was created by using gazebo model editor.  

#### wheeled_model_enhanced
It's a robot with four wheels on its chassis. The front of the robot contains a rod with a camera on it. In the middle of the top of the chassis there is a sensor block. The sensor block contains:  
1. IMU sensor
2. GPS sensor

The camera is providing a video stream. The wheels of the robot can be controlled to set a desired speed. Topics that can be used to get the robot information:
1. `cmd_vel` (`geometry_msgs/msg/Twist`) - to move the robot around
2. `/camera/image_raw` (`sensor_msgs/msg/Image`) - to see what in the front of the robot
3. `/camera_pos_cmd` (`std_msgs/msg/Float64`) - to move the camera around
4. `/imu` (`sensor_msgs/msg/Imu`) - to get IMU data
5. `/wheeled_model_enhanced/navsat` (`sensor_msgs/msg/NavSatFix`) - to get the robot GPS data
6. `/model/wheeled_model_enhanced/odometry` (`nav_msgs/msg/Odometry`) - for the robot odometry
7. `/model/wheeled_model_enhanced/tf` (`geometry_msgs/msg/PoseArray`) - for the robot positioning (**Note**: the robot is moving in the ENU basis, so the `tf` info can't be used to understand where the robot is directly)  

#### waypoint
it's a stick with a sphere on it and with a square base. The sphere contains the GPS sensor. Created to simplify a debugging of the robot moving.  

Available topics:
1. `/waypoint/navsat` (`sensor_msgs/msg/NavSatFix`) - to get the waypoint GPS data  

#### Plane ground
Endless plane ground.  

### World
Contains `wheeled_model_enhanced`, `waypoint`, and `plane ground`.  

### Launch file
Here parameters for nodes can be specified so they will be used in every launch.  

### Nodes
#### reach_goal_action_server
Turning the robot to the waypoint. The result - the camera of the robot is facing the waypoint.  
Available parameters:
1. `angle_threshold`. Float. Default value is `0.05`. Radians.  
2. `distance_threshold`. Float. Default value is `0.1`. Meters.  
3. `max_angle_acceleration`. Float. Default value is `2.0`. Radians/sec^2.  
4. `max_angle_velocity`. Float. Default value is `1.0`. Radians/sec.  
5. `robot_imu_twist`. Difference between IMU x-axe and the robot front. Float. Default value is `1.5707963267948966`. Radians.  
6. `max_acceleration`. Max linear acceleration. Float. Default value is `1.0`. Meters/sec^2.  
7. `max_velocity`. Max linear velocity. Float. Default value is `10.0`. Meters/sec.  
8. `robot_length_in_m`. Because the GPS sensor is placed in the middle of the robot, the distance between this sensor and the robot front must be noted for achieving the waypoint gracefully. Float. Default value is `1.5`. Meters.  

#### reach_goal_action_client
Using to trigger reach_goal_action_server.  
