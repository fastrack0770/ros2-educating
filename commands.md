### Creating package
`ros2 pkg create package_name --build-type ament_cmake`

### Building
`colcon build` - command for simple build  

`colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo` - command for build with debug info  

`colcon build --symlink-install` - command for build with symlinks - to easy take changes from .xml and other files  

### Running
`ros2 run udemy_ros2_pkg publisher` - command for simple run  
Form is `ros2 run`, next package name, next node name(?)  

`ros2 run --prefix 'gdbserver localhost:3000' udemy_ros2_pkg publisher` - command for run with debug info (using gdb server)  

`ros2 run udemy_ros2_pkg publisher --ros-args --log-level debug` - command for run with debug output (all output)  


`ros2 run udemy_ros2_pkg publisher --ros-args --log-level hello_world_sub_node:=debug` - command for run with debug output (only one node)  

### Interacting with ROS2 nodes
`ros2 topic list` - to see topic where data is going right now  
`ros2 topic echo /topic_name` - to see data in the topic  

`ros2 param list` - to see params for all nodes that is running  
`ros2 param get /node_name param_name` - to see what value is in the param  
`ros2 param set /node_name param_name value` - to place new value in the param. Note that a type must match to that in the parameter  

`ros2 launch package_name launch_file_name` - to run launch file for package  

### GAZEBO SIMULATION
`export IGN_GAZEBO_RESOURCE_PATH=/absolute/path/to/models/dir` - to make Gazebo to be able to find models  

`ign gazebo -v 4 -r world_name.sdf` - to start gazebo simulation. `-v 4` means max log, `-r` means from beginning  

`ros2 run ros_gz_bridge parameter_bridge /lidar2@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan --ros-args -r /lidar2:=/laser_scan` - to make a bridge for lazer  

`ros2 run ros_ign_bridge parameter_bridge /camera@sensor_msgs/msg/Image[ignition.msgs.Image --ros-args -r /camera:=/camera/image_raw` - to make a bridge for camera  

`ign gazebo -v 4 -r world_name_.sdf --gui-config config_name.config` - to run gazebo simulation with specific gui config  

`code /usr/share/ignition/ignition-gazebo6/worlds` - to open all worlds with simulations examples