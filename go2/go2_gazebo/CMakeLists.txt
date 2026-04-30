cmake_minimum_required(VERSION 3.5)
project(go2_gazebo)

add_compile_options(-std=c++17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(gazebo_ros2_control REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(sensor_msgs REQUIRED)

set(dependencies
  rclcpp
  urdf
  gazebo_ros2_control
  gazebo_ros
  sensor_msgs
)


#############
## Install ##
#############

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(${dependencies})

ament_package()
