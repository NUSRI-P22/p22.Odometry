cmake_minimum_required(VERSION 3.5)
project(p22_odometry)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(serial REQUIRED)

add_executable(p22_odometry src/p22_odometry.cpp)
ament_target_dependencies(p22_odometry rclcpp std_msgs nav_msgs serial)


## Include headers
target_include_directories(${PROJECT_NAME} PRIVATE include)

install(TARGETS
  p22_odometry
  DESTINATION lib/${PROJECT_NAME})

ament_package()
