import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    current_pkg = FindPackageShare('sparkfun_rtk_express')

    return launch.LaunchDescription([
        # Mapviz node
        DeclareLaunchArgument(
            'mapviz_config',
            default_value=PathJoinSubstitution([current_pkg, 'config', 'mapviz.mvc']),
            description='Mapviz config file'
        ),
        Node(
            package='mapviz',
            executable='mapviz',
            name='mapviz',
            output='screen',
            parameters=[{
                'config': LaunchConfiguration('mapviz_config'),
                'autosave': False
            }]
        ),

        # Origin Initialization
        Node(
            package='swri_transform_util',
            executable='initialize_origin.py',
            name='initialize_origin',
            parameters=[{
                'local_xy_frame': 'map',
                'local_xy_navsatfix_topic': 'gps/fix/origin',
                'local_xy_origin': 'pitt',  # Change to 'pitt' for fixed origin
                'local_xy_origins': """[{
                    'name': 'pitt',
                    'latitude': 13.651400,
                    'longitude': 100.494429,
                    'altitude': 0.0,
                    'heading': 0.0
                }]"""
            }]
        ),

        # # GPS Datum Node
        # Node(
        #     package='mapviz_gps',
        #     executable='gps_datum.py',
        #     name='gps_datum',
        #     output='screen'
        # ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="swri_transform",
            arguments=["0", "0", "0", "0", "0", "0", "map", "origin"]
        )
    ])