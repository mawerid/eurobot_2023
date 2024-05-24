from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    
    default_robot_launch_path = PathJoinSubstitution(
        [FindPackageShare('tank'), 'launch', 'default_tank.launch.py']
    )
    return LaunchDescription([

        DeclareLaunchArgument(
            name='Pico 1', 
            default_value='/dev/ttyACM0',
            description='Tank Pico 1'
        ),

        DeclareLaunchArgument(
            name='Pico 2', 
            default_value='/dev/ttyACM1',
            description='Tank Pico 2'
        ),

        Node(
            package='tank',
            executable='reboot',
            name='reboot',
        ),

        Node(
            package='tank',
            executable='tank_camera',
            name='tank_camera',
        ),
        
        Node(
            package='tank',
            executable='screen',
            name='screen',
        ),

       IncludeLaunchDescription(
            PythonLaunchDescriptionSource(default_robot_launch_path),
            launch_arguments={
                'Pico 1': LaunchConfiguration("Pico 1"),
                'Pico 2': LaunchConfiguration("Pico 2")
            }.items()
        ),
    ])