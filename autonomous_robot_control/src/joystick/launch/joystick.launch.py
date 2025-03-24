from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    joy_params = os.path.join(get_package_share_directory('autonomous_robot_description'),'config','joystick.yaml')

    joy_node = Node(
            package='joy',
            executable='joy_node',
            parameters=[joy_params, {'use_sim_time': use_sim_time}],
         )

    # using my joy node
    teleop_node = Node(
            package='joystick',
            executable='teleop_joy_node',
            name='teleop_node',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[('/cmd_vel','/cmd_vel')]
         )
    
    # using ros2 teleop twist joy node
    # teleop_node = Node(
    #         package='teleop_twist_joy',
    #         executable='teleop_node',
    #         name='teleop_node',
    #         parameters=[joy_params, {'use_sim_time': use_sim_time}]  # remember to add ,
    #         #remappings=[('/cmd_vel','/cmd_vel')]
    #      )

    # Node to add a timestamp to incoming /cmd_vel_unstamped messages and publish them to /cmd_vel
    # twist_stamper = Node( 
    #         package='twist_stamper',
    #         executable='twist_stamper',
    #         parameters=[{'use_sim_time': use_sim_time}],
    #         remappings=[('/cmd_vel_in','/cmd_vel_'),
    #                     ('/cmd_vel_out','/diff_cont/cmd_vel')])


    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true'),
        joy_node,
        teleop_node
        # twist_stamper       
    ])