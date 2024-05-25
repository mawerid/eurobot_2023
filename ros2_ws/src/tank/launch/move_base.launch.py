from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        Node(
            package='tank',
            executable='move_base',
            name='move_base',
        ),

        Node(
            package='tank',
            executable='move_planning',
            name='move_planning',
        ),
        Node(
            package='tank',
            executable='scanner',
            name='scanner',
        ),
        Node(
            package='tank',
            executable='after_the_stop',
            name='after_the_stop',
        ),

    ])
