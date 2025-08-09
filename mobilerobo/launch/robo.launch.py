import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directory
    pkg_share = FindPackageShare(package='rostut').find('rostut')

    # Path to the URDF xacro file
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')

    # World file path
    world_file = os.path.join(pkg_share, 'worlds', 'empty.world')
    
    # Declare launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    x = DeclareLaunchArgument(
        'x',
        default_value='0.0',
        description='X position of the robot in the world'
    )
    
    y = DeclareLaunchArgument(
        'y',
        default_value='0.0',
        description='Y position of the robot in the world'
    )
    
    # Robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=[Command(['xacro ', urdf_file])]
    )
    
    # rviz node
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'robot.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    return LaunchDescription([
        use_sim_time,
        x,
        y,
        robot_state_publisher,
        rviz_node,
    ])




