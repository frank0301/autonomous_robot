cmake_minimum_required(VERSION 3.5)
project(sensors_package)

# Use C++ standard
set(CMAKE_CXX_STANDARD 14)

# Find required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Configuration for the Python file of the camera module
install(PROGRAMS
  src/camera_rgb_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/launch
)


# Define the IMU executable
# add_executable(mpu6050_driver src/imu_ws/driver_mpu6050.cpp)
# ament_target_dependencies(mpu6050_driver rclcpp sensor_msgs std_msgs)

# Install the IMU executable
# install(TARGETS mpu6050_driver
#   DESTINATION lib/${PROJECT_NAME}
# )

# Define the package as a ROS 2 package
ament_package()
