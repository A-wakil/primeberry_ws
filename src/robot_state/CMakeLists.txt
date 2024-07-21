cmake_minimum_required(VERSION 3.8)
project(robot_state)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(primeberry_odom_pub src/primeberry_odom_pub.cpp)
ament_target_dependencies(primeberry_odom_pub rclcpp std_msgs nav_msgs geometry_msgs tf2_ros)

install(TARGETS
  primeberry_odom_pub
  DESTINATION lib/${PROJECT_NAME})

ament_package()
