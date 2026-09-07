cmake_minimum_required(VERSION 3.22)
project(mios_ros2_runtime)

include(CTest)

find_package(ament_cmake REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(mios_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Eigen3 REQUIRED)

set(MIOS_CONTROL_TYPES_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../src/utils/include")
if(NOT EXISTS "${MIOS_CONTROL_TYPES_INCLUDE_DIR}/mios/control/control_types.hpp")
  # The production Docker image imports this intentionally small, neutral
  # contract without copying any legacy transport or libfranka source.
  set(MIOS_CONTROL_TYPES_INCLUDE_DIR "/ws/src/mios_control_types/include")
endif()
if(NOT EXISTS "${MIOS_CONTROL_TYPES_INCLUDE_DIR}/mios/control/control_types.hpp")
  message(FATAL_ERROR "Cannot find mios/control/control_types.hpp")
endif()

set(MIOS_ROBOT_BACKEND_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../src/panda/include")
set(MIOS_UTILS_TYPES_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../src/utils/include")
if(NOT EXISTS "${MIOS_ROBOT_BACKEND_INCLUDE_DIR}/mios/panda/robot_backend.hpp")
  set(MIOS_ROBOT_BACKEND_INCLUDE_DIR "/ws/src/mios_core_contract/include")
  set(MIOS_UTILS_TYPES_INCLUDE_DIR "/ws/src/mios_core_contract/include")
endif()
if(NOT EXISTS "${MIOS_ROBOT_BACKEND_INCLUDE_DIR}/mios/panda/robot_backend.hpp" OR
   NOT EXISTS "${MIOS_UTILS_TYPES_INCLUDE_DIR}/mios/utils/types.hpp")
  message(FATAL_ERROR "Cannot find the MIOS Core RobotBackend contract headers")
endif()

add_library(mios_ros2_runtime_translation STATIC
  src/arm_command_translation.cpp
  src/gripper_state_translation.cpp
  src/robot_parameter_snapshot.cpp
  src/robot_state_translation.cpp)
target_compile_features(mios_ros2_runtime_translation PUBLIC cxx_std_17)
set_target_properties(mios_ros2_runtime_translation PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(mios_ros2_runtime_translation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${MIOS_CONTROL_TYPES_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(mios_ros2_runtime_translation sensor_msgs)
target_link_libraries(mios_ros2_runtime_translation Eigen3::Eigen)

add_library(mios_ros2_runtime_backend SHARED
  src/ros2_arm_command_dispatcher.cpp
  src/ros2_core_robot_backend.cpp
  src/ros2_gripper_client.cpp
  src/ros2_robot_parameter_client.cpp
  src/ros2_robot_backend.cpp)
target_compile_features(mios_ros2_runtime_backend PUBLIC cxx_std_17)
target_include_directories(mios_ros2_runtime_backend PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${MIOS_CONTROL_TYPES_INCLUDE_DIR}>
  $<BUILD_INTERFACE:${MIOS_ROBOT_BACKEND_INCLUDE_DIR}>
  $<BUILD_INTERFACE:${MIOS_UTILS_TYPES_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mios_ros2_runtime_backend mios_ros2_runtime_translation Eigen3::Eigen)
ament_target_dependencies(mios_ros2_runtime_backend franka_msgs mios_msgs rclcpp rclcpp_action sensor_msgs std_srvs)

add_executable(mios_runtime_node src/mios_runtime_node.cpp)
target_compile_features(mios_runtime_node PUBLIC cxx_std_17)
target_link_libraries(mios_runtime_node mios_ros2_runtime_backend)
ament_target_dependencies(mios_runtime_node rclcpp)

install(DIRECTORY include/ DESTINATION include)
install(FILES
  "${MIOS_CONTROL_TYPES_INCLUDE_DIR}/mios/control/control_types.hpp"
  "${MIOS_CONTROL_TYPES_INCLUDE_DIR}/mios/control/robot_parameter_translation.hpp"
  DESTINATION include/mios/control)
install(FILES
  "${MIOS_ROBOT_BACKEND_INCLUDE_DIR}/mios/panda/robot_backend.hpp"
  DESTINATION include/mios/panda)
install(FILES
  "${MIOS_UTILS_TYPES_INCLUDE_DIR}/mios/utils/types.hpp"
  DESTINATION include/mios/utils)
install(TARGETS mios_ros2_runtime_translation mios_ros2_runtime_backend
  EXPORT export_mios_ros2_runtime
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS mios_runtime_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  add_executable(test_arm_command_translation test/test_arm_command_translation.cpp)
  target_compile_features(test_arm_command_translation PRIVATE cxx_std_17)
  target_link_libraries(test_arm_command_translation mios_ros2_runtime_translation)
  add_test(NAME test_arm_command_translation COMMAND test_arm_command_translation)

  add_executable(test_robot_state_translation test/test_robot_state_translation.cpp)
  target_compile_features(test_robot_state_translation PRIVATE cxx_std_17)
  target_link_libraries(test_robot_state_translation mios_ros2_runtime_translation)
  add_test(NAME test_robot_state_translation COMMAND test_robot_state_translation)

  add_executable(test_gripper_state_translation test/test_gripper_state_translation.cpp)
  target_compile_features(test_gripper_state_translation PRIVATE cxx_std_17)
  target_link_libraries(test_gripper_state_translation mios_ros2_runtime_translation)
  add_test(NAME test_gripper_state_translation COMMAND test_gripper_state_translation)

  add_executable(test_robot_parameter_snapshot test/test_robot_parameter_snapshot.cpp)
  target_compile_features(test_robot_parameter_snapshot PRIVATE cxx_std_17)
  target_link_libraries(test_robot_parameter_snapshot mios_ros2_runtime_translation)
  add_test(NAME test_robot_parameter_snapshot COMMAND test_robot_parameter_snapshot)

  add_executable(test_mios_parameter_snapshot_translation
    test/test_mios_parameter_snapshot_translation.cpp)
  target_compile_features(test_mios_parameter_snapshot_translation PRIVATE cxx_std_17)
  target_link_libraries(test_mios_parameter_snapshot_translation mios_ros2_runtime_translation)
  add_test(NAME test_mios_parameter_snapshot_translation
    COMMAND test_mios_parameter_snapshot_translation)

  add_executable(test_ros2_gripper_client test/test_ros2_gripper_client.cpp)
  target_compile_features(test_ros2_gripper_client PRIVATE cxx_std_17)
  target_link_libraries(test_ros2_gripper_client mios_ros2_runtime_backend)
  ament_target_dependencies(test_ros2_gripper_client rclcpp)
  add_test(NAME test_ros2_gripper_client COMMAND test_ros2_gripper_client)

  add_executable(test_ros2_core_robot_backend test/test_ros2_core_robot_backend.cpp)
  target_compile_features(test_ros2_core_robot_backend PRIVATE cxx_std_17)
  target_link_libraries(test_ros2_core_robot_backend mios_ros2_runtime_backend)
  ament_target_dependencies(test_ros2_core_robot_backend rclcpp)
  add_test(NAME test_ros2_core_robot_backend COMMAND test_ros2_core_robot_backend)

  add_executable(test_ros2_arm_command_dispatcher
    test/test_ros2_arm_command_dispatcher.cpp)
  target_compile_features(test_ros2_arm_command_dispatcher PRIVATE cxx_std_17)
  target_link_libraries(test_ros2_arm_command_dispatcher mios_ros2_runtime_backend)
  ament_target_dependencies(test_ros2_arm_command_dispatcher rclcpp)
  add_test(NAME test_ros2_arm_command_dispatcher COMMAND test_ros2_arm_command_dispatcher)

  add_executable(test_ros2_robot_parameter_client test/test_ros2_robot_parameter_client.cpp)
  target_compile_features(test_ros2_robot_parameter_client PRIVATE cxx_std_17)
  target_link_libraries(test_ros2_robot_parameter_client mios_ros2_runtime_backend)
  ament_target_dependencies(test_ros2_robot_parameter_client rclcpp)
  add_test(NAME test_ros2_robot_parameter_client COMMAND test_ros2_robot_parameter_client)
endif()

ament_export_targets(export_mios_ros2_runtime HAS_LIBRARY_TARGET)
ament_export_dependencies(franka_msgs mios_msgs rclcpp rclcpp_action sensor_msgs std_srvs Eigen3)
ament_export_include_directories(include)

ament_package()
