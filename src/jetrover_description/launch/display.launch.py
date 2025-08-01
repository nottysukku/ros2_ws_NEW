import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    compiled = os.environ.get('need_compile', 'False')  # Safely get env var

    namespace = LaunchConfiguration('namespace', default='')
    use_namespace = LaunchConfiguration('use_namespace', default='false')
    frame_prefix = LaunchConfiguration('frame_prefix', default='')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    frame_prefix_arg = DeclareLaunchArgument('frame_prefix', default_value=frame_prefix)
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value=use_sim_time)
    namespace_arg = DeclareLaunchArgument('namespace', default_value=namespace)
    use_namespace_arg = DeclareLaunchArgument('use_namespace', default_value=use_namespace)

    if compiled == 'True':
        jetrover_description_package_path = get_package_share_directory('jetrover_description')
    else:
        jetrover_description_package_path = '/home/ubuntu/ros2_ws/src/simulations/jetrover_description'

    urdf_path = os.path.join(jetrover_description_package_path, 'urdf/jetrover.xacro')
    rviz_config_file = os.path.join(jetrover_description_package_path, 'rviz/view.rviz')

    robot_description = Command(['xacro ', urdf_path])

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'frame_prefix': frame_prefix,
            'use_sim_time': use_sim_time
        }],
    )

    rviz_node = ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config_file],
        output='screen'
    )

    delay_rviz_node = TimerAction(
        period=5.0,
        actions=[rviz_node],
    )

    return LaunchDescription([
        frame_prefix_arg,
        use_sim_time_arg,
        namespace_arg,
        use_namespace_arg,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        delay_rviz_node,
    ])
