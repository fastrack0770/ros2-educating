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
udemy_ros2_pkg_path = get_package_share_directory("udemy_ros2_pkg")
simulation_world_file_path = Path(
    udemy_ros2_pkg_path, "worlds", "wheeled_model_world.sdf"
)
simulation_models_file_path = Path(udemy_ros2_pkg_path, "models")

# simulation = IncludeLaunchDescription(
#     PythonLaunchDescriptionSource(
#         launch_file_path=Path(
#             ignition_ros_package_path, "launch", "ign_gazebo.launch.py"
#         ),
#     ),
#     launch_arguments={"ign_args": "-r " + simulation_world_file_path}.items(),
# )

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
                    "/model/wheeled_model/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist"
                ],
                remappings=[("/model/wheeled_model/cmd_vel", "cmd_vel")],  # Remapping
                output="screen",
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=simulation, on_exit=[EmitEvent(event=Shutdown())]
                )
            ),
        ]
    )
