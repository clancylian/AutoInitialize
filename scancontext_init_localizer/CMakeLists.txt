cmake_minimum_required(VERSION 3.8)
project(scancontext_init_localizer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(PCL REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/EstimateInitialPose.srv"
  DEPENDENCIES geometry_msgs
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_library(scancontext_core
  src/scan_context.cpp
)

ament_target_dependencies(scancontext_core
  rclcpp
  sensor_msgs
  pcl_ros
  pcl_conversions
)

target_link_libraries(scancontext_core
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(sc_map_builder_node
  src/sc_map_builder_node.cpp
)

ament_target_dependencies(sc_map_builder_node
  rclcpp
  message_filters
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  pcl_ros
  pcl_conversions
)

target_link_libraries(sc_map_builder_node
  scancontext_core
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(sc_ndt_initializer_node
  src/sc_ndt_initializer_node.cpp
)

ament_target_dependencies(sc_ndt_initializer_node
  rclcpp
  message_filters
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  pcl_ros
  pcl_conversions
  std_srvs
)

target_link_libraries(sc_ndt_initializer_node
  scancontext_core
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
)

rosidl_target_interfaces(sc_map_builder_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

rosidl_target_interfaces(sc_ndt_initializer_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  scancontext_core
  sc_map_builder_node
  sc_ndt_initializer_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
