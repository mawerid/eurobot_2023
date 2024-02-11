import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    pkg_ros_gz_sim_demos = get_package_share_directory('ros_gz_sim_demos')
    return LaunchDescription([
        # Launch gazebo
        ExecuteProcess(
            cmd=[
                'gz', 'sim', '-r',
                os.path.join(
                    pkg_ros_gz_sim_demos,
                    'models',
                    'double_pendulum_model.sdf'
                )
            ]
        ),
        # Launch a bridge to forward tf and joint states to ros2
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/world/default/model/double_pendulum_with_base0/joint_state@'
                'sensor_msgs/msg/JointState[gz.msgs.Model',
                '/model/double_pendulum_with_base0/pose@'
                'tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
            ],
            remappings=[
                ('/model/double_pendulum_with_base0/pose', '/tf'),
                ('/world/default/model/double_pendulum_with_base0/joint_state', '/joint_states')
            ]
        ),
        # Launch rviz
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg_ros_gz_sim_demos, 'rviz', 'tf_bridge.rviz')]
        )
    ])