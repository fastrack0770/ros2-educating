from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

# Retrieving path information
from ament_index_python.packages import get_package_share_directory
from os.path import join as Path

# Utilizing launch files from other packages
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import SetEnvironmentVariable

# React on event
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown

# Path Variables
ignition_ros_package_path = get_package_share_directory("ros_ign_gazebo")
wheeled_model_enhanced_path = get_package_share_directory("wheeled_model_enhanced")
simulation_world_file_path = Path(
    wheeled_model_enhanced_path, "worlds", "wheeled_model_enhanced_world.sdf"
)
simulation_models_file_path = Path(wheeled_model_enhanced_path, "models")

simulation = ExecuteProcess(
    cmd=["ign", "gazebo", "-r", simulation_world_file_path], output="screen"
)


def generate_launch_description():
    return LaunchDescription(
        [
            SetEnvironmentVariable(
                name="IGN_GAZEBO_RESOURCE_PATH", value=simulation_models_file_path
            ),
            simulation,
            Node(
                package="ros_ign_bridge",
                executable="parameter_bridge",
                arguments=[
                    "/model/wheeled_model_enhanced/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist",
                    "/camera@sensor_msgs/msg/Image[ignition.msgs.Image",
                    "/camera_pos_cmd@std_msgs/msg/Float64@ignition.msgs.Double",
                    "/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU",
                    "/navsat@sensor_msgs/msg/NavSatFix@ignition.msgs.NavSat"
                ],
                remappings=[
                    ("/model/wheeled_model_enhanced/cmd_vel", "cmd_vel"),
                    ("/camera", "/camera/image_raw"),
                ],
                output="screen",
            ),
            # Actually doesnt work. There are three copies of Node above that are created, and OnProcessExist kills only one of them
            # Btw if stop the simulation by Ctrl+C, all three copies will stop gracefully
            # Reference to issue: https://github.com/fastrack0770/ros2-educating/issues/1
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=simulation, on_exit=[EmitEvent(event=Shutdown())]
                )
            ),
        ]
    )
