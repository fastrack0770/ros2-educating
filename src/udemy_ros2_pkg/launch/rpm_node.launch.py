from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="udemy_ros2_pkg",
            executable="rpm_publisher",
            name="rpm_pub_node",
            parameters=[
                {"rpm_val": 70.0}
            ]
        ),
        Node(
            package="udemy_ros2_pkg",
            executable="speed_publisher",
            name="speed_pub_node",
            parameters=[
                {"wheel_rad": 2.0}
            ]
        ),
        ExecuteProcess(
            cmd=['ros2', 'topic', 'list'],
            output='screen'
        )
    ])