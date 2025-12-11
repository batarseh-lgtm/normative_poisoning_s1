#!/usr/bin/env python3
"""
Launch file for baseline (non-poisoned) missions
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for baseline missions."""
    
    # Get package directory
    pkg_dir = get_package_share_directory('s1_attack_simulation')
    config_file = os.path.join(pkg_dir, 'config', 'mission_params.yaml')
    sectors_file = os.path.join(pkg_dir, 'config', 'sectors.yaml')
    
    # Include Gazebo + PX4
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('s1_attack_simulation'),
                'launch',
                'gazebo_px4.launch.py'
            ])
        ])
    )
    
    # UAV Agent Node (baseline mode)
    uav_agent_node = Node(
        package='s1_attack_simulation',
        executable='uav_agent_node',
        name='uav_agent_node',
        output='screen',
        parameters=[
            config_file,
            {
                'mode': 'baseline',
                'llm_model': 'mock',
                'context_storage_path': '~/.ros2_s1_attack/context_baseline'
            }
        ]
    )
    
    # Mission Executor Node
    mission_executor_node = Node(
        package='s1_attack_simulation',
        executable='mission_executor',
        name='mission_executor',
        output='screen',
        parameters=[
            config_file,
            sectors_file
        ]
    )
    
    # Metrics Collector Node
    metrics_collector_node = Node(
        package='s1_attack_simulation',
        executable='metrics_collector',
        name='metrics_collector',
        output='screen',
        parameters=[{
            'output_dir': '~/.ros2_s1_attack/metrics',
            'mission_name': 'baseline'
        }]
    )
    
    return LaunchDescription([
        gazebo_launch,
        # Delay agent nodes to allow Gazebo/PX4 to start
        TimerAction(
            period=5.0,
            actions=[
                uav_agent_node,
                mission_executor_node,
                metrics_collector_node
            ]
        )
    ])
