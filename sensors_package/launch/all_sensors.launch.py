from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Paths to the IMU and LiDAR launch files
    imu_launch_file = os.path.join(
        get_package_share_directory('mpu6050driver'), 'launch', 'mpu6050driver_launch.py.py'
    )

    lidar_launch_file = os.path.join(
        get_package_share_directory('lslidar_driver'), 'launch', 'lslidar_cx_launch.py'
    )

    # Include the IMU launch file
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(imu_launch_file)
    )

    # Include the LiDAR launch file
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar_launch_file)
    )

    return LaunchDescription([
        imu_launch,
        lidar_launch
    ])