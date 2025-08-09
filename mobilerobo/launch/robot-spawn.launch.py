import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directories
    pkg_share = FindPackageShare('mobilerobo').find('mobilerobo')
    robot_description_dir = get_package_share_directory('robobase_description')
    urdf_xacro = os.path.join(robot_description_dir, 'models', 'robo.urdf.xacro')
    urdf_tmp = '/tmp/robot.urdf'
    sdf_tmp = '/tmp/robot.sdf'

    # Launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    robot_name = DeclareLaunchArgument(
        'robot_name', default_value='robot',
        description='Robot name for spawning'
    )
    x = DeclareLaunchArgument(
        'x', default_value='0.0', description='X position'
    )
    y = DeclareLaunchArgument(
        'y', default_value='0.0', description='Y position'
    )
    z = DeclareLaunchArgument(
        'z', default_value='0.3', description='Z position'
    )
    yaw = DeclareLaunchArgument(
        'yaw', default_value='0.0', description='Yaw orientation'
    )

    # Convert xacro to SDF using gz sdf tool
    convert_to_sdf = ExecuteProcess(
        cmd=['bash', '-c', f"xacro '{urdf_xacro}' -o '{urdf_tmp}' && gz sdf -p '{urdf_tmp}' > '{sdf_tmp}'"],
        output='screen'
    )

    # Spawn SDF
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', LaunchConfiguration('robot_name'),
            '-file', sdf_tmp,
            '-allow_renaming', 'true',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-Y', LaunchConfiguration('yaw')
        ],
        output='screen'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': Command(['xacro ', urdf_xacro])
        }]
    )

    return LaunchDescription([
        use_sim_time,
        robot_name,
        x,
        y,
        z,
        yaw,
        convert_to_sdf,
        robot_state_publisher,
        spawn_entity,
    ])
