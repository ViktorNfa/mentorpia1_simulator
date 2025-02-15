import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, TextSubstitution

def generate_launch_description():
    pkg_mentorpia1 = get_package_share_directory('mentorpia1_simulator')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    mesh_path = os.path.join(pkg_mentorpia1, 'meshes')
    current_paths = os.environ.get('GAZEBO_MODEL_PATH', '')
    os.environ['GAZEBO_MODEL_PATH'] = mesh_path + ':' + current_paths

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Xacro -> URDF
    robot_description_path = os.path.join(pkg_mentorpia1, 'urdf', 'mentorpi.xacro')
    robot_description_config = xacro.process_file(robot_description_path)
    robot_description = {'robot_description': robot_description_config.toxml()}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    # World
    world = LaunchConfiguration("world")
    world_path = os.path.join(pkg_mentorpia1, 'worlds', 'empty_world.sdf')
    declare_world_cmd = DeclareLaunchArgument(
        "world",
        default_value=world_path,
        description="Path to the world file."
    )
    world_str_path = [TextSubstitution(text='-r '), world]

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': world_str_path}.items(),
    )

    # Spawn SDF
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

    # Bridge /clock and /cmd_vel => geometry_msgs/Twist <-> gz.msgs.Twist
    clock_and_cmd_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
        ],
        output='screen'
    )

    # Node that converts AckermannDrive -> Twist, so you can publish Ackermann on /cmd_ackermann
    ackermann_to_twist = Node(
        package='mentorpia1_simulator',
        executable='ackermann_to_twist',  # if you set it up as a console script
        # or use a direct path if not installed as console script
        name='ackermann_to_twist',
        output='screen',
        # optional param if you want to set wheelbase
        parameters=[{'wheelbase': 1.0}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        declare_world_cmd,
        gazebo,
        spawn,
        clock_and_cmd_vel_bridge,
        robot_state_publisher,
        ackermann_to_twist,
    ])