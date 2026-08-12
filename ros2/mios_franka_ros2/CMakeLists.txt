cmake_minimum_required(VERSION 3.16)
project(mios_franka_ros2)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(franka_semantic_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(realtime_tools REQUIRED)

add_library(mios_franka_controllers SHARED
  src/cartesian_controllers.cpp
)
target_compile_features(mios_franka_controllers PUBLIC cxx_std_17)
target_include_directories(mios_franka_controllers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(mios_franka_controllers
  controller_interface
  franka_semantic_components
  geometry_msgs
  pluginlib
  rclcpp
  realtime_tools
)
target_link_libraries(mios_franka_controllers Eigen3::Eigen)

pluginlib_export_plugin_description_file(controller_interface mios_franka_controllers.xml)

install(TARGETS mios_franka_controllers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/mios_franka DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(
  controller_interface
  franka_semantic_components
  geometry_msgs
  realtime_tools
)
ament_export_libraries(mios_franka_controllers)
ament_package()
