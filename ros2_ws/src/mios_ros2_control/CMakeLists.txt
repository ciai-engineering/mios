cmake_minimum_required(VERSION 3.22)
project(mios_ros2_control)

include(CTest)

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(franka_semantic_components REQUIRED)
find_package(mios_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_msgs REQUIRED)

add_library(mios_effort_controller SHARED
  src/mios_control_core.cpp
  src/mios_robot_model_broadcaster.cpp
  src/mios_effort_controller.cpp
  src/mios_zero_effort_diagnostic_controller.cpp
  src/mios_velocity_control_core.cpp
  src/mios_joint_velocity_controller.cpp
  src/mios_cartesian_velocity_control_core.cpp
  src/mios_cartesian_velocity_controller.cpp
  src/mios_position_control_core.cpp
  src/mios_joint_position_controller.cpp
  src/mios_cartesian_pose_control_core.cpp
  src/mios_cartesian_pose_controller.cpp)
target_compile_features(mios_effort_controller PUBLIC cxx_std_17)
target_include_directories(mios_effort_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(mios_effort_controller
  controller_interface franka_msgs franka_semantic_components mios_msgs pluginlib rclcpp realtime_tools std_msgs)

pluginlib_export_plugin_description_file(controller_interface mios_controllers.xml)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS mios_effort_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(FILES mios_controllers.xml DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY test DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)

  set(MIOS_CONTROL_CORE_TESTS
    test_mios_control_core
    test_mios_velocity_control_core
    test_mios_cartesian_velocity_control_core
    test_mios_position_control_core
    test_mios_cartesian_pose_control_core)

  foreach(test_target IN LISTS MIOS_CONTROL_CORE_TESTS)
    add_executable(${test_target} test/${test_target}.cpp)
    target_compile_features(${test_target} PRIVATE cxx_std_17)
    target_link_libraries(${test_target} mios_effort_controller)
    add_test(NAME ${test_target} COMMAND ${test_target})
  endforeach()

  # This starts controller_manager against the seven-joint GenericSystem
  # fixture. It verifies that the plugin loads/configures through the same
  # lifecycle used on hardware, and that the shipped commissioning lock
  # rejects activation without ever opening an FCI connection.
  add_launch_test(test/test_fake_hardware_lifecycle.launch.py TIMEOUT 60)
endif()

ament_package()
