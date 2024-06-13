cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(vrx_ros)

find_package(ament_cmake REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(gz-msgs9 REQUIRED)
find_package(gz-transport12 REQUIRED)
set(GZ_TRANSPORT_VER ${gz-transport12_VERSION_MAJOR})
find_package(rclcpp REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(my_robot_interfaces REQUIRED)

#Python
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

add_executable(optical_frame_publisher src/optical_frame_publisher.cc)
ament_target_dependencies(optical_frame_publisher
  geometry_msgs
  rclcpp
  tf2
  tf2_ros
  sensor_msgs
)

add_executable(pose_tf_broadcaster src/pose_tf_broadcaster.cc)
ament_target_dependencies(pose_tf_broadcaster
  geometry_msgs
  rclcpp
  tf2
  tf2_ros
  tf2_msgs
)

add_executable(ASV_control src/ASV_control.cpp)
ament_target_dependencies(ASV_control
  geometry_msgs
  rclcpp
  tf2_ros
  ros_gz_interfaces
  nav_msgs
  tf2_geometry_msgs
  tf2_msgs
  tf2
)

add_executable(controller_asv_3 src/controller_asv_3.cpp)
ament_target_dependencies(controller_asv_3
  geometry_msgs
  rclcpp
  tf2_ros
  ros_gz_interfaces
  nav_msgs
  tf2_geometry_msgs
  tf2_msgs
  tf2
)

add_executable(controller_asv_2 src/controller_asv_2.cpp)
ament_target_dependencies(controller_asv_2
  geometry_msgs
  rclcpp
  tf2_ros
  ros_gz_interfaces
  nav_msgs
  tf2_geometry_msgs
  tf2_msgs
  tf2
)

add_executable(controller_asv_1 src/controller_asv_1.cpp)
ament_target_dependencies(controller_asv_1
  geometry_msgs
  rclcpp
  tf2_ros
  ros_gz_interfaces
  nav_msgs
  tf2_geometry_msgs
  tf2_msgs
  tf2
)

add_executable(controller_asv_4 src/controller_asv_4.cpp)
ament_target_dependencies(controller_asv_4
  geometry_msgs
  rclcpp
  tf2_ros
  ros_gz_interfaces
  nav_msgs
  tf2_geometry_msgs
  tf2_msgs
  tf2
)
# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/BF_1_main.py
  scripts/BF_2_main.py
  scripts/TDOA_3D.py
  scripts/PF.py
  scripts/plots.py
  scripts/Beamformer_DS_MVDR.py
  DESTINATION lib/${PROJECT_NAME}
)

# Resources
install(TARGETS
  ASV_control
  optical_frame_publisher
  pose_tf_broadcaster
  controller_asv_1
  controller_asv_2
  controller_asv_3
  controller_asv_4
  DESTINATION lib/${PROJECT_NAME})

  
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
