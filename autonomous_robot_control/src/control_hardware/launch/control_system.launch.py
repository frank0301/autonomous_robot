import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Define paths for URDF and Twist Mux config
    urdf_path = PathJoinSubstitution([
        FindPackageShare("autonomous_robot_description"), "urdf", "autonomous_robot.urdf.xacro"
    ])
    twist_mux_params = os.path.join(
        get_package_share_directory("autonomous_robot_description"), "config", "twist_mux.yaml"
    )

    #controller_params_file = os.path.join(get_package_share_directory("autonomous_robot_description"),'config','joint_controller.yaml')
    
    joy_params = os.path.join(get_package_share_directory('autonomous_robot_description'),'config','joystick.yaml')

    # Declare launch arguments
    declare_urdf_path = DeclareLaunchArgument(
        name="urdf_path",
        default_value=urdf_path,
        description="Path to the robot URDF file"
    )

    # Robot State Publisher Node
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": Command(["xacro ", LaunchConfiguration("urdf_path")]),
            "use_sim_time": False
        }]
    )

    joint_state_publisher = Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    name='joint_state_publisher',
    output="screen",
    parameters=[{"use_sim_time":False}]
    )

    # # Twist Mux Node
    # twist_mux = Node(
    #     package="twist_mux",
    #     executable="twist_mux",
    #     parameters=[twist_mux_params, {"use_sim_time": False}],
    #     remappings=[("/cmd_vel_out", "/cmd_vel_muxed")],
    # )


    # # Teleop Joy Node
    # teleop_joy_node = Node(
    #     package="joystick",  
    #     executable="teleop_joy_node", 
    #     name="teleop_joy_node",
    #     output="screen",
    #     parameters=[{"use_sim_time": False}]
    # )

    # # CmdVel to PWM Node
    # cmd_vel_to_pwm_node = Node(
    #     package="control_hardware",
    #     executable="cmd2pwm",
    #     name="cmd_vel_to_pwm_node",
    #     output="screen",
    #     parameters=[{"use_sim_time": False, "wheel_base": 0.5}]
    # )

    # # PWM to GPIO Node
    # pwm_to_gpio_node = Node(
    #     package="control_hardware",
    #     executable="pwm2GPIO",
    #     name="pwm_to_gpio_node",
    #     output="screen",
    #     parameters=[{"use_sim_time": False}]
    # )


    # Teleop Joy Node
    teleop_joy_node = Node(
        package="control_hardware",  # Replace with your package name
        executable="joy_control_node",  # Replace with your executable name
        name="joy_control_node",
        output="screen",
        parameters=[{"use_sim_time": False}]
    )

    # CmdVel to PWM Node
    cmd_vel_to_gpio_node = Node(
        package="control_hardware",
        executable="cmd2gpio",
        name="cmd2gpio_node",
        output="screen",
        parameters=[{"use_sim_time": False}]
    )
    # Return the LaunchDescription with all nodes and arguments
    return LaunchDescription([
        declare_urdf_path,
        robot_state_publisher,
        joint_state_publisher,
        teleop_joy_node,
        cmd_vel_to_gpio_node,
        # twist_mux,
        # teleop_joy_node,
        # cmd_vel_to_pwm_node,
        # pwm_to_gpio_node
    ])
