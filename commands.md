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

Command for create package
`ros2 pkg create $1 --build-type ament_cmake`