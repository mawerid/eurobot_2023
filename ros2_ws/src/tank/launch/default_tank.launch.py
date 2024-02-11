from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    description_launch_path = PathJoinSubstitution(
        [FindPackageShare('tank'), 'launch', 'description_tank.launch.py']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='base_serial_port', 
            #default_value='/dev/ttyACM0',
            default_value='/dev/ttyS0',
            description='Linorobot Base Serial Port'
        ),
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            output='screen',
            arguments=['serial', '--dev', LaunchConfiguration("base_serial_port")]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch_path)
        ),
    ])