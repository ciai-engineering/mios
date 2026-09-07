set(MODULE_NAME controller_pipeline)

set(${MODULE_NAME}_SRC
    src/cart_torque_pipeline.cpp
    src/cart_velocity_pipeline.cpp
    src/cart_joint_torque_pipeline.cpp
    src/joint_torque_pipeline.cpp
    src/joint_velocity_pipeline.cpp
    src/joint_position_pipeline.cpp
    src/joint_position_pipeline_math.cpp
    src/mios_algorithm_executor.cpp
    src/null_pipeline.cpp
    )

set(${MODULE_NAME}_HEADER_FILES
    include/${PROJECT_NAME}/${MODULE_NAME}/controller_pipeline.hpp
    include/${PROJECT_NAME}/${MODULE_NAME}/cart_torque_pipeline.hpp
    include/${PROJECT_NAME}/${MODULE_NAME}/cart_joint_torque_pipeline.hpp
    include/${PROJECT_NAME}/${MODULE_NAME}/cart_velocity_pipeline.hpp
    include/${PROJECT_NAME}/${MODULE_NAME}/joint_torque_pipeline.hpp
    include/${PROJECT_NAME}/${MODULE_NAME}/joint_velocity_pipeline.hpp
    include/${PROJECT_NAME}/${MODULE_NAME}/joint_position_pipeline.hpp
    include/${PROJECT_NAME}/${MODULE_NAME}/mios_algorithm_executor.hpp
    include/${PROJECT_NAME}/${MODULE_NAME}/null_pipeline.hpp
    )

add_library(${MODULE_NAME}
    ${${MODULE_NAME}_SRC}
    ${${MODULE_NAME}_HEADER_FILES}
    )


add_library(${PROJECT_NAME}::${MODULE_NAME} ALIAS ${MODULE_NAME})

target_include_directories(${MODULE_NAME}
    PRIVATE
    src
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )

find_package(cntr_aic REQUIRED)
find_package(cntr_force REQUIRED)
find_package(cntr_joint_var_imp REQUIRED)
find_package(cntr_nullsp_proj REQUIRED)
find_package(cntr_mux REQUIRED)
find_package(conv_vel2pose REQUIRED)
find_package(Eigen3 REQUIRED)

target_link_libraries(${MODULE_NAME}
    PRIVATE
    ${PROJECT_NAME}_project_options
    ${PROJECT_NAME}_project_warnings
    ${PROJECT_NAME}_project_deps
    mirmi_cpp_utils::math
    PUBLIC
    cntr_aic::cntr_aic
    cntr_force::cntr_force
    cntr_joint_var_imp::cntr_joint_var_imp
    cntr_nullsp_proj::cntr_nullsp_proj
    cntr_mux::cntr_mux
    conv_vel2pose::conv_vel2pose
    ${PROJECT_NAME}::safety
    ${PROJECT_NAME}::utils
    Eigen3::Eigen
    )

set_target_properties(${MODULE_NAME} PROPERTIES PUBLIC_HEADER "${${MODULE_NAME}_HEADER_FILES}")

apply_module_options(${MODULE_NAME})

if(ENABLE_TESTING)
    add_executable(test_joint_position_pipeline
        test/test_joint_position_pipeline.cpp
        src/joint_position_pipeline_math.cpp)
    target_compile_features(test_joint_position_pipeline PRIVATE cxx_std_17)
    target_include_directories(test_joint_position_pipeline PRIVATE
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/src/utils/include)
    target_link_libraries(test_joint_position_pipeline PRIVATE
        ${PROJECT_NAME}::${PROJECT_NAME}_project_deps
        Eigen3::Eigen)
    add_test(NAME test_joint_position_pipeline COMMAND test_joint_position_pipeline)
endif()
