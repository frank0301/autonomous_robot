import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Define paths for URDF and RViz config
    urdf_path = PathJoinSubstitution([
        FindPackageShare("autonomous_robot_description"), "urdf", "autonomous_robot.urdf.xacro"
    ])

    rviz_config_path = PathJoinSubstitution([
        FindPackageShare("autonomous_robot_description"), "rviz", "urdf_config.rviz"
    ])

    # Declare arguments
    declare_urdf_path = DeclareLaunchArgument(
        name="urdf_path",
        default_value=urdf_path,
        description="Path to the robot URDF file"
    )

    declare_rviz_config_path = DeclareLaunchArgument(
        name="rviz_config_path",
        default_value=rviz_config_path,
        description="Path to the RViz configuration file"
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": Command(["xacro ", LaunchConfiguration("urdf_path")]),
            "use_sim_time": True
        }]
    )

    # Gazebo Launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("aws_robomaker_small_warehouse_world"), "launch", "no_roof_small_warehouse.launch.py"
            ])
        ]),
        launch_arguments={
            "world_name": PathJoinSubstitution([
                FindPackageShare("aws_robomaker_small_warehouse_world"), "worlds", "no_roof_small_warehouse.world"
            ])
        }.items()
    )

    # Joint State Publisher
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    # Spawn the robot entity in Gazebo
    spawn_robot_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=["-topic", "robot_description", "-entity", "autonomous_robot"]
    )

    # # Spawning the differential controller
    spawn_diff_controller = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["diff_cont"]
    )

    # Spawning the joint board controller
    spawn_joint_board_controller = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["joint_broad"]
    )

    # RViz Launch
    rviz_launch = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config_path")],
        parameters=[{"use_sim_time": True}]
    )

    # Twist Mux changing the topic name, instead of /cmd_vel it will be /diff_cont/cmd_vel_unstamped 
    # and giving priority.
    twist_mux_params = os.path.join(get_package_share_directory('autonomous_robot_description'),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
        )
    # cart_odom= Node(
    #             package='calc_odom',           # Name of the package
    #             executable='cartographer_odom', # Name of the executable
    #             name='cartographer_odom',      # Optional name for the node
    #             output='screen',               # Output logs to the screen
    #             parameters=[]                  # Add any parameters if needed
    #         )

    return LaunchDescription([
        declare_urdf_path,
        declare_rviz_config_path,
        robot_state_publisher,
        gazebo_launch,
        joint_state_publisher,
        spawn_robot_entity,
        spawn_diff_controller,
        spawn_joint_board_controller,
        rviz_launch,
        twist_mux,
        cart_odom

    ])
