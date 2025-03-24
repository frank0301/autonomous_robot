from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sensors_package',  
            executable='camera_rgb_publisher.py', 
            name='camera_rgb_publisher',
            output='screen'
 #           parameters=[{
  #              'image_size': [640,480],
   #             'camera_frame_id': 'camera_link_optical'
    #            }]
        ),
    ])
