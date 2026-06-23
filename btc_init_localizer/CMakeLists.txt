cmake_minimum_required(VERSION 3.8)
project(btc_init_localizer)

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
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(PCL REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)
find_package(TBB REQUIRED COMPONENTS tbb)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/EstimateInitialPose.srv"
  DEPENDENCIES geometry_msgs
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_library(btc_core
  src/BTC.cpp
)

ament_target_dependencies(btc_core
  rclcpp
  sensor_msgs
  pcl_ros
  pcl_conversions
)

target_link_libraries(btc_core
  ${PCL_LIBRARIES}
  TBB::tbb
)

add_library(btc_database
  src/btc_database.cpp
)

ament_target_dependencies(btc_database
  rclcpp
  sensor_msgs
  pcl_ros
  pcl_conversions
)

target_link_libraries(btc_database
  btc_core
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  TBB::tbb
)

add_executable(btc_map_builder_node
  src/btc_map_builder_node.cpp
)

ament_target_dependencies(btc_map_builder_node
  rclcpp
  message_filters
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  pcl_ros
  pcl_conversions
  visualization_msgs
)

target_link_libraries(btc_map_builder_node
  btc_core
  btc_database
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  TBB::tbb
)

add_executable(btc_ndt_initializer_node
  src/btc_ndt_initializer_node.cpp
)

ament_target_dependencies(btc_ndt_initializer_node
  rclcpp
  message_filters
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  pcl_ros
  pcl_conversions
  visualization_msgs
  std_srvs
)

target_link_libraries(btc_ndt_initializer_node
  btc_core
  btc_database
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  TBB::tbb
)

rosidl_target_interfaces(btc_map_builder_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

rosidl_target_interfaces(btc_ndt_initializer_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  btc_core
  btc_database
  btc_map_builder_node
  btc_ndt_initializer_node
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
