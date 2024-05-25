from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    move_robot_launch_path = PathJoinSubstitution(
        [FindPackageShare('tank'), 'launch', 'move_base.launch.py'])
    bringup_robot_launch_path = PathJoinSubstitution(
        [FindPackageShare('tank'), 'launch', 'tank_bringup.launch.py'])
    
    return LaunchDescription([
        Node(
            package='tank',
            executable='main_algorithm',
            name='main_algorithm',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(move_robot_launch_path),

        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_robot_launch_path),

        ),
        
    
    ])