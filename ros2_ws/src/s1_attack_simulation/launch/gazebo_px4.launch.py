#!/usr/bin/env python3
"""
Launch Gazebo with PX4 SITL and MAVROS
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for Gazebo + PX4 + MAVROS."""
    
    # Get package directory
    pkg_dir = get_package_share_directory('s1_attack_simulation')
    world_file = os.path.join(pkg_dir, 'worlds', 'sectors.world')
    
    # Declare arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_file,
        description='Gazebo world file'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Gazebo server (headless)
    gzserver = ExecuteProcess(
        cmd=['gzserver',
             '--verbose',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             LaunchConfiguration('world')],
        output='screen'
    )
    
    # Gazebo client (GUI)
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )
    
    # Note: PX4 SITL needs to be launched separately or via a custom script
    # This is because PX4 setup varies widely depending on installation
    # Users should run: make px4_sitl gazebo (from PX4-Autopilot directory)
    
    # MAVROS node
    # Note: This assumes MAVROS is installed and configured
    # You may need to adjust the fcu_url based on your PX4 setup
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        output='screen',
        parameters=[{
            'fcu_url': 'udp://:14540@localhost:14557',
            'gcs_url': '',
            'target_system_id': 1,
            'target_component_id': 1,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    return LaunchDescription([
        world_arg,
        use_sim_time_arg,
        gzserver,
        gzclient,
        # mavros_node,  # Uncomment when MAVROS is properly configured
    ])
