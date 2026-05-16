"""
Launch the fully autonomous package recovery mission.

Starts: servo_node, camera_node, bucket_detector, autonomous_mission.

Usage:
    ros2 launch pies_mission autonomous.launch.py

Then trigger the mission:
    ros2 topic pub --once /mission/go std_msgs/msg/Empty "{}"
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from pies_mission import mission_config as cfg


def generate_launch_description():
    return LaunchDescription([

        Node(
            package='pies_servo',
            executable='servo_node',
            name='servo_node',
            output='screen',
        ),

        Node(
            package='camera_ros',
            executable='camera_node',
            name='camera_node',
            output='screen',
        ),

        Node(
            package='pies_vision',
            executable='bucket_detector',
            name='bucket_detector',
            output='screen',
            parameters=[{'target_color': cfg.TARGET_COLOR}],
        ),

        Node(
            package='pies_mission',
            executable='autonomous_mission',
            name='autonomous_mission',
            output='screen',
        ),
    ])
