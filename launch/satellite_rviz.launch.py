import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Path to the RViz config file
    rviz_config_path = os.path.join(
        os.path.dirname(__file__), '..', 'config', 'view.rviz'
    )

    return LaunchDescription([
        # Launch GNSS data publisher
        Node(
            package='sparkfun_rtk_express',  
            executable='GPSRTKpublisher.py',
            name='gnss_publisher',
            output='screen'
        ),

        # Launch NavSatFix converter
        Node(
            package='sparkfun_rtk_express', 
            executable='navsatfix.py',
            name='navsatfix_converter',
            output='screen'
        ),

        # Launch RViz2 with the custom configuration
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        )
    ])
