import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, Command
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directory
    jetrover_description_package_path = get_package_share_directory('jetrover_description')
    
    # Define paths
    urdf_path = os.path.join(jetrover_description_package_path, 'urdf/jetrover.xacro')
    world_file = os.path.join(jetrover_description_package_path, 'worlds/empty.world')
    rviz_config_file = os.path.join(jetrover_description_package_path, 'rviz/view.rviz')
    
    # Verify if world file exists, if not create a default empty world
    if not os.path.exists(world_file):
        os.makedirs(os.path.dirname(world_file), exist_ok=True)
        with open(world_file, 'w') as f:
            f.write("""<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin
      filename="gz-sim-physics-system"
      name="gz::sim::systems::Physics">
    </plugin>
    <plugin
      filename="gz-sim-sensors-system"
      name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin
      filename="gz-sim-scene-broadcaster-system"
      name="gz::sim::systems::SceneBroadcaster">
    </plugin>
    <plugin
      filename="gz-sim-user-commands-system"
      name="gz::sim::systems::UserCommands">
    </plugin>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
""")
    
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
    use_rviz = LaunchConfiguration('use_rviz')
    frame_prefix = LaunchConfiguration('frame_prefix')
    
    # Robot description with required environment variables
    robot_description = Command([
        'xacro ', urdf_path,
        ' LIDAR_TYPE:=A1',
        ' MACHINE_TYPE:=JetRover_Mecanum'
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
        
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz')
        
    declare_frame_prefix_cmd = DeclareLaunchArgument(
        'frame_prefix',
        default_value='',
        description='Prefix for tf frames')

    declare_robot_pose_args = [
        DeclareLaunchArgument('x_pose', default_value='0.0', description='x component of initial position [m]'),
        DeclareLaunchArgument('y_pose', default_value='0.0', description='y component of initial position [m]'),
        DeclareLaunchArgument('z_pose', default_value='0.1', description='z component of initial position [m]'),
        DeclareLaunchArgument('roll', default_value='0.0', description='roll angle of initial orientation [rad]'),
        DeclareLaunchArgument('pitch', default_value='0.0', description='pitch angle of initial orientation [rad]'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='yaw angle of initial orientation [rad]'),
    ]

    # Start Gazebo with absolute path to world file
    start_gazebo_cmd = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '4', world_file],
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
            'robot_description': robot_description,
            'frame_prefix': frame_prefix
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
            'use_sim_time': use_sim_time,
            'source_list': ['/controller_manager/joint_states'],
            'rate': 20.0
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
    
    # RViz
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(use_rviz)
    )
    
    # Launch RViz after a delay to ensure Gazebo and the robot are loaded
    delayed_rviz_cmd = TimerAction(
        period=5.0,
        actions=[rviz_cmd],
    )

    return LaunchDescription([
        # Declare arguments
        declare_use_sim_time_cmd,
        declare_world_cmd,
        declare_gui_cmd,
        declare_headless_cmd,
        declare_namespace_cmd,
        declare_use_namespace_cmd,
        declare_use_rviz_cmd,
        declare_frame_prefix_cmd,
        *declare_robot_pose_args,
        
        # Launch nodes
        start_gazebo_cmd,
        robot_state_publisher_cmd,
        spawn_entity_cmd,
        joint_state_publisher_cmd,
        bridge_cmd,
        delayed_rviz_cmd,
    ])