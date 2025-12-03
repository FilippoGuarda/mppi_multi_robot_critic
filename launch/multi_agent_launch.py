# nav2_interaction_critic/launch/multi_agent_launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """Launch multi-agent navigation with interaction-aware control."""
    
    # Get package directories
    nav2_pkg = get_package_share_directory('nav2_bringup')
    interaction_critic_pkg = get_package_share_directory('nav2_interaction_critic')
    
    # Launch robot 1 with Nav2
    robot1_namespace = 'robot1'
    robot1_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'namespace': robot1_namespace,
            'use_namespace': 'True',
            'slam': 'True',
            'map': os.path.join(interaction_critic_pkg, 'maps', 'arena.yaml'),
            'use_sim_time': 'True',
            'params_file': os.path.join(
                interaction_critic_pkg, 'config', 'controller_params.yaml'
            ),
        }.items(),
    )
    
    # Launch robot 2 with Nav2
    robot2_namespace = 'robot2'
    robot2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'namespace': robot2_namespace,
            'use_namespace': 'True',
            'slam': 'True',
            'map': os.path.join(interaction_critic_pkg, 'maps', 'arena.yaml'),
            'use_sim_time': 'True',
            'params_file': os.path.join(
                interaction_critic_pkg, 'config', 'controller_params.yaml'
            ),
        }.items(),
    )
    
    # Static transforms for multi-robot scenario
    static_tf_robot1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'robot1/map'],
    )
    
    static_tf_robot2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['2', '0', '0', '0', '0', '0', 'map', 'robot2/map'],
    )
    
    ld = LaunchDescription([
        robot1_launch,
        robot2_launch,
        static_tf_robot1,
        static_tf_robot2,
    ])
    
    return ld
