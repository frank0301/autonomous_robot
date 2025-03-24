cmake_minimum_required(VERSION 3.8)
project(joystick)

# Set compiler warnings for GCC and Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies for C++ and Python
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robot_common_interfaces REQUIRED)

# Include directories for C++
include_directories(include)

# Add the C++ library (if needed)
add_library(joystick
  src/joy_compute_control.cpp
)

# Add executables for the C++ nodes
add_executable(teleop_joy_node src/teleop_joy_node.cpp)
ament_target_dependencies(teleop_joy_node rclcpp geometry_msgs sensor_msgs)

add_executable(prototype_joy_driver src/prototype_joy_driver.cpp)
ament_target_dependencies(prototype_joy_driver rclcpp robot_common_interfaces sensor_msgs)
target_link_libraries(prototype_joy_driver joystick)

# Install the C++ executables
install(TARGETS
  teleop_joy_node
  prototype_joy_driver
  DESTINATION lib/${PROJECT_NAME}
)


# Install header files for C++
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
