import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directories
    pkg_nav2_dir = get_package_share_directory('nav2_bringup')
    pkg_meca_description = get_package_share_directory('meca_description')
    config_dir = os.path.join(pkg_meca_description, 'config')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart = LaunchConfiguration('autostart', default='true')
    rviz_config = LaunchConfiguration('rviz_config', 
        default=os.path.join(pkg_nav2_dir, 'rviz', 'nav2_default_view.rviz'))

    # Main Nav2 bringup
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'map': os.path.join(config_dir, 'my_map.yaml'),
            'params_file': os.path.join(config_dir, 'nav2_params.yaml'),
        }.items()
    )

    # Additional nodes
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[os.path.join(config_dir, 'amcl_params.yaml')],
        remappings=[('scan', '/scan')]  # Adjust if your laser topic is different
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
            
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically startup the nav2 stack'),
            
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(pkg_nav2_dir, 'rviz', 'nav2_default_view.rviz'),
            description='Full path to RVIZ config file'),
            
        nav2_launch,
        amcl_node,
        static_tf_node,
        rviz_node
    ])
