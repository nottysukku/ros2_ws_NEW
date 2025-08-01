import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    # Get package directory
    jetrover_description_package_path = get_package_share_directory('jetrover_description')
    
    # Define paths
    urdf_path = os.path.join(jetrover_description_package_path, 'urdf/jetrover.xacro')
    world_file = os.path.join(jetrover_description_package_path, 'worlds/empty.world')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    headless = LaunchConfiguration('headless')
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    roll = LaunchConfiguration('roll')
    pitch = LaunchConfiguration('pitch')
    yaw = LaunchConfiguration('yaw')
    
    # Robot description with required environment variables
    robot_description = Command([
        'xacro ', urdf_path,
        ' LIDAR_TYPE:=ouster',
        ' MACHINE_TYPE:=jetrover'
    ])

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=world_file,
        description='Full path to world model file to load')

    declare_gui_cmd = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Set to "false" to run headless.')

    declare_headless_cmd = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Whether to execute headless')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    declare_robot_pose_args = [
        DeclareLaunchArgument('x_pose', default_value='0.0', description='x component of initial position [m]'),
        DeclareLaunchArgument('y_pose', default_value='0.0', description='y component of initial position [m]'),
        DeclareLaunchArgument('z_pose', default_value='0.1', description='z component of initial position [m]'),
        DeclareLaunchArgument('roll', default_value='0.0', description='roll angle of initial orientation [rad]'),
        DeclareLaunchArgument('pitch', default_value='0.0', description='pitch angle of initial orientation [rad]'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='yaw angle of initial orientation [rad]'),
    ]

    # Start Gazebo (simplified approach)
    start_gazebo_cmd = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '4', world],
        output='screen'
    )

    # Robot State Publisher
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }]
    )

    # Spawn robot in Gazebo
    spawn_entity_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'jetrover',
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-R', roll,
            '-P', pitch,
            '-Y', yaw
        ],
        output='screen'
    )

    # Joint State Publisher
    joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    # Bridge between Gazebo and ROS topics
    bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model'
        ],
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    return LaunchDescription([
        # Declare arguments
        declare_use_sim_time_cmd,
        declare_world_cmd,
        declare_gui_cmd,
        declare_headless_cmd,
        declare_namespace_cmd,
        declare_use_namespace_cmd,
        *declare_robot_pose_args,
        
        # Launch nodes
        start_gazebo_cmd,
        robot_state_publisher_cmd,
        spawn_entity_cmd,
        joint_state_publisher_cmd,
        bridge_cmd,
    ])