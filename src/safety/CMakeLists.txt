set(MODULE_NAME safety)

set(${MODULE_NAME}_SRC
    src/cartesian_velocity_damping.cpp
    src/velocity_walls.cpp
    src/virtual_cube.cpp
    src/virtual_joint_walls.cpp
    )

set(${MODULE_NAME}_HEADER_FILES
    include/${PROJECT_NAME}/safety_stage_1/velocity_walls.hpp
    include/${PROJECT_NAME}/safety_stage_1/safety_module_stage_1.hpp
    include/${PROJECT_NAME}/safety_stage_2/cartesian_velocity_damping.hpp
    include/${PROJECT_NAME}/safety_stage_2/virtual_cube.hpp
    include/${PROJECT_NAME}/safety_stage_2/virtual_joint_walls.hpp
    include/${PROJECT_NAME}/safety_stage_2/safety_module_stage_2.hpp
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
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    )

find_package(virtual_cube REQUIRED)
find_package(virtual_walls_joint REQUIRED)
find_package(Eigen3 REQUIRED)

target_link_libraries(${MODULE_NAME}
    PRIVATE
    ${PROJECT_NAME}::${PROJECT_NAME}_project_options
    ${PROJECT_NAME}::${PROJECT_NAME}_project_warnings
    ${PROJECT_NAME}::${PROJECT_NAME}_project_deps
    mirmi_cpp_utils::math
    PUBLIC
    virtual_cube::virtual_cube
    virtual_walls_joint::virtual_walls_joint
    ${PROJECT_NAME}::utils
    Eigen3::Eigen
    )

apply_module_options(${MODULE_NAME})
