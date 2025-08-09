import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directory
    pkg_share = FindPackageShare(package='mobilerobo').find('mobilerobo')
    robot_discription_directory = get_package_share_directory('robobase_description')

    # Path to the URDF xacro file
    urdf_file = os.path.join(robot_discription_directory, 'models', 'robo.urdf.xacro')

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
    
    # Correct: Set robot_description parameter using Command substitution
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': Command(['xacro ', urdf_file])
        }]
    )
    
    # rviz node
    rviz_config_file = os.path.join(pkg_share, 'config', 'robot.rviz')
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




