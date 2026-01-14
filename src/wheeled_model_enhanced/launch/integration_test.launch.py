from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch.actions import ExecuteProcess

# Retrieving path information
from ament_index_python.packages import get_package_share_directory
from os.path import join as Path

# Utilizing launch files from other packages
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import SetEnvironmentVariable

# React on event
from launch.actions import RegisterEventHandler, EmitEvent, TimerAction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown, matches_action
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import State, Transition
import launch_ros

# Path Variables
ignition_ros_package_path = get_package_share_directory("ros_ign_gazebo")
wheeled_model_enhanced_path = get_package_share_directory("wheeled_model_enhanced")
simulation_world_file_path = Path(
    wheeled_model_enhanced_path, "worlds", "wheeled_model_enhanced_world.sdf"
)
simulation_models_file_path = Path(wheeled_model_enhanced_path, "models")


def generate_launch_description():
    simulation = ExecuteProcess(
        cmd=["ign", "gazebo", "-r", simulation_world_file_path], output="screen"
    )

    lifecycle_server = LifecycleNode(
        package="wheeled_model_enhanced",
        executable="reach_goal_action_server",
        name="reach_goal_action_server_node",
        namespace="",
        parameters=[
            {"angle_threshold": 0.05},
            {"distance_threshold": 1.0},
            {"max_angle_acceleration": 2.0},
            {"max_angle_velocity": 2.0},
            {"robot_imu_twist": 1.5707963267948966},  # in rads
            {"max_acceleration": 10.0},
            {"max_velocity": 10.0},
            {"robot_length_in_m": 1.5},
        ],
    )

    lifecycle_server_configure = EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=matches_action(lifecycle_server),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    lifecycle_server_activate = RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=lifecycle_server,
            goal_state="inactive",
            entities=[
                TimerAction(
                    period=5.0,
                    actions=[
                        EmitEvent(
                            event=launch_ros.events.lifecycle.ChangeState(
                                lifecycle_node_matcher=matches_action(lifecycle_server),
                                transition_id=Transition.TRANSITION_ACTIVATE,
                            )
                        ),
                    ],
                )
            ],
        )
    )

    integration_test = Node(
        package="wheeled_model_enhanced",
        executable="integration_test",
        name="integration_test_node",
    )

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
                    "/wheeled_model_enhanced/navsat@sensor_msgs/msg/NavSatFix@ignition.msgs.NavSat",
                    "/waypoint/navsat@sensor_msgs/msg/NavSatFix@ignition.msgs.NavSat",
                    "/model/wheeled_model_enhanced/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
                    "/model/wheeled_model_enhanced/tf@geometry_msgs/msg/PoseArray[ignition.msgs.Pose_V",
                    "/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
                ],
                remappings=[
                    ("/model/wheeled_model_enhanced/cmd_vel", "cmd_vel"),
                    ("/camera", "/camera/image_raw"),
                ],
                output="screen",
            ),
            lifecycle_server,
            lifecycle_server_configure,
            lifecycle_server_activate,
            RegisterEventHandler(
                OnStateTransition(
                    target_lifecycle_node=lifecycle_server,
                    goal_state="active",
                    entities=[integration_test]
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=integration_test,
                    on_exit=[EmitEvent(event=Shutdown())],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=simulation, on_exit=[EmitEvent(event=Shutdown())]
                )
            ),
        ]
    )
