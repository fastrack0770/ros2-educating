Command for simple build:
`colcon build`

Command for build with debug info:
`colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo`

Command for build with symlinks - to easy take changes from .xml and other files
`colcon build --symlink-install`

Command for simple run:
`ros2 run udemy_ros2_pkg publisher`
Form is `ros2 run`, next package name, next node name(?)

Command for run with debug info (using gdb server)
`ros2 run --prefix 'gdbserver localhost:3000' udemy_ros2_pkg publisher`

Command for run with debug output (all output):
`ros2 run udemy_ros2_pkg publisher --ros-args --log-level debug`

Command for run with debug output (only one node):
`ros2 run udemy_ros2_pkg publisher --ros-args --log-level hello_world_sub_node:=debug`

Command for create package
`ros2 pkg create $1 --build-type ament_cmake`

`ros2 topic list` - to see topic where data is going right now
`ros2 topic echo /topic_name` - to see data in the topic

`ros2 param list` - to see params for all nodes that is running
`ros2 param get /node_name param_name` - to see what value is in the param
`ros2 param set /node_name param_name value` - to place new value in the param. Note that a type must match to that in the parameter

`ros2 launch package_name launch_file_name` - to run launch file for package