cmake_minimum_required(VERSION 3.5)
project(inpipe_robot)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(xacro REQUIRED)
find_package(rviz2 REQUIRED)
find_package(robot_localization REQUIRED)
find_package(OpenCV REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(inpipe_robot_interfaces REQUIRED)

find_package(PCL REQUIRED COMPOENETS common io)
find_package(pcl_conversions REQUIRED)

include_directories(include)

add_executable(ipir_bringup
  src/ipir_bringup.cpp
)

add_executable(odometry
  src/ipir_odometry.cpp
)

add_executable(mapper
  src/ipir_mapper.cpp
)

add_executable(kinema
  src/position_kinema_node.cpp
)

add_executable(tilt_bringup
  src/ipir_joint_positions.cpp
)

add_executable(wheel_bringup
  src/ipir_wheel_control.cpp
)

add_executable(read_write_node
  src/read_write_node.cpp
)

add_executable(locomotion
  src/ipir_locomotions.cpp
)

# ament_target_dependencies(${PROJECT_NAME}
#   rclcpp
#   std_msgs
#   sensor_msgs
#   geometry_msgs
#   nav_msgs
#   dynamixel_sdk
#   inpipe_robot_interfaces
# )

ament_target_dependencies(ipir_bringup
  rclcpp
  rclcpp_action
  dynamixel_sdk
  geometry_msgs
  std_srvs
  inpipe_robot_interfaces
)

ament_target_dependencies(odometry
  rclcpp
  tf2
  tf2_geometry_msgs
  nav_msgs
  sensor_msgs
  inpipe_robot_interfaces
)

ament_target_dependencies(mapper
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  inpipe_robot_interfaces
  tf2
  tf2_geometry_msgs
  pcl_conversions
)

ament_target_dependencies(kinema
  rclcpp
)

ament_target_dependencies(tilt_bringup
  rclcpp
  dynamixel_sdk
  inpipe_robot_interfaces
)

ament_target_dependencies(wheel_bringup
  rclcpp
  geometry_msgs
  dynamixel_sdk
)

ament_target_dependencies(read_write_node
  rclcpp
  geometry_msgs
  dynamixel_sdk
  inpipe_robot_interfaces
)

ament_target_dependencies(locomotion
  rclcpp
  geometry_msgs
  dynamixel_sdk
  inpipe_robot_interfaces
)

target_link_libraries(mapper
  ${PCL_LIBRARIES}
)

target_link_libraries(kinema
  ${OpenCV_LIBS}
)

install(TARGETS
  # ${PROJECT_NAME}
  ipir_bringup
  odometry
  mapper
  kinema
  tilt_bringup
  wheel_bringup
  read_write_node
  locomotion
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  src
  launch
  meshes
  models
  urdf
  config
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
