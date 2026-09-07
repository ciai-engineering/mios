set(MODULE_NAME mios_ros2_core_runtime)

find_package(rclcpp REQUIRED)
find_package(mios_ros2_runtime REQUIRED)

add_executable(${MODULE_NAME}
    src/mios_ros2_core_node.cpp)

target_compile_features(${MODULE_NAME} PRIVATE cxx_std_17)
target_link_libraries(${MODULE_NAME}
    PRIVATE
    ${PROJECT_NAME}::core
    rclcpp::rclcpp
    mios_ros2_runtime::mios_ros2_runtime_backend)

install(TARGETS ${MODULE_NAME}
    RUNTIME DESTINATION bin)

apply_module_options(${MODULE_NAME})
