import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, TextSubstitution

def generate_launch_description():
    # Get package directories
    pkg_mentorpia1 = get_package_share_directory('mentorpia1_simulator')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    mesh_path = os.path.join(pkg_mentorpia1, 'meshes')
    current_paths = os.environ.get('GAZEBO_MODEL_PATH', '')
    os.environ['GAZEBO_MODEL_PATH'] = mesh_path + ':' + current_paths

    # Use simulated time by default
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Process the mentorpiA1 xacro file to generate the robot description (URDF)
    robot_description_path = os.path.join(pkg_mentorpia1, 'urdf', 'mentorpi.xacro')
    robot_description_config = xacro.process_file(robot_description_path)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    # World file (update the filename as needed)
    world = LaunchConfiguration("world")
    world_path = os.path.join(pkg_mentorpia1, 'worlds', 'empty_world.sdf')
    declare_world_cmd = DeclareLaunchArgument(
        "world",
        default_value=world_path,
        description="Path to the world file."
    )
    world_str_path = [TextSubstitution(text='-r '), world]

    # Include the Gazebo (Gz) simulation launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': world_str_path}.items(),
    )

    # Spawn the mentorpiA1 robot using its SDF file.
    # (This SDF should include the AckermannSteering plugin configuration.)
    mentorpi_sdf_path = os.path.join(pkg_mentorpia1, 'urdf', 'mentorpia1.sdf')
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'mentorpia1',
            '-file', mentorpi_sdf_path,
            '-z', '0.1',
            '-x', '0.0',
        ],
        output='screen'
    )

    # Bridge for converting between ROS 2 and Gazebo messages.
    # Here we bridge the /cmd_vel topic as an Ackermann drive command.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '/cmd_vel@ackermann_msgs/msg/AckermannDrive@gz.msgs.AckermannDrive',
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='If true, use simulated clock'),
        declare_world_cmd,
        gazebo,
        spawn,
        bridge,
        robot_state_publisher,
    ])