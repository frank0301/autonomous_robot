cmake_minimum_required(VERSION 3.8)
project(autonomous_robot_description)

# Set policy CMP0009 to NEW to suppress the warning
if(POLICY CMP0009)
  cmake_policy(SET CMP0009 NEW)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install directories
install(
  DIRECTORY config launch rviz urdf meshes
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
