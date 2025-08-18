#!/usr/bin/env python3

"""
Launch file for JetRover simulation with joint controllers
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Get the path to the SDF model
    pkg_jetrover_description = FindPackageShare('jetrover_description')
    sdf_file = PathJoinSubstitution([
        pkg_jetrover_description, 
        'models', 
        'jetrover', 
        'jetrover_world1.sdf'
    ])
    
    # Launch Gazebo with the world
    gazebo_launch = ExecuteProcess(
        cmd=[
            'gz', 'sim', 
            sdf_file,
            '-v', '4'  # Verbose logging
        ],
        name='gazebo_sim',
        output='screen'
    )
    
    # Launch the joint controller demo
    demo_launch = ExecuteProcess(
        cmd=[
            'python3',
            PathJoinSubstitution([
                pkg_jetrover_description,
                'scripts',
                'joint_controller_demo.py'
            ])
        ],
        name='joint_demo',
        output='screen'
    )
    
    return LaunchDescription([
        gazebo_launch,
        # Uncomment the next line to auto-start the demo
        # demo_launch,
    ])
