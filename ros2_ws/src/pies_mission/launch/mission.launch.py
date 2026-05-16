"""
Launch the full visual-centering mission stack.

Starts: servo_node, camera_node, bucket_detector, visual_centering.
All parameters come from pies_mission/mission_config.py — edit that file
before running, no command-line flags needed.

Usage:
    ros2 launch pies_mission mission.launch.py
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
            executable='visual_centering',
            name='visual_centering',
            output='screen',
            parameters=[{
                'cam_offset_x_px':    cfg.CAM_OFFSET_X_PX,
                'cam_offset_y_px':    cfg.CAM_OFFSET_Y_PX,
                'kp_x':               cfg.KP_X,
                'kp_y':               cfg.KP_Y,
                'max_vel':            cfg.MAX_VEL,
                'center_threshold_px': cfg.CENTER_THRESHOLD_PX,
                'confirm_frames':     cfg.CONFIRM_FRAMES,
                'descent_speed':      cfg.DESCENT_SPEED,
                'grab_angle':         cfg.GRAB_ANGLE,
                'release_angle':      cfg.RELEASE_ANGLE,
                'grab_alt':           cfg.GRAB_ALT,
                'blind_grab_s':       cfg.BLIND_GRAB_S,
                'cam_rot_deg':        cfg.CAM_ROT_DEG,
            }],
        ),
    ])
