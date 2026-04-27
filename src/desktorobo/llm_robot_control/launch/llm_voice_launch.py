from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mobile_robot_control',
            executable='mobile_robot_control_node',
            name='robot_controll',
        ),
        Node(
            package='llm_robot_control',
            executable='llm_controller_node',
            name='llm_controller',
            output='screen',
        ),
        Node(
            package='llm_robot_control',
            executable='voice_controller_node',
            name='voice_controller',
            output='screen',
        ),
    ])
