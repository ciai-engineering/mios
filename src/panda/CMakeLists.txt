set(MODULE_NAME panda)

set(${MODULE_NAME}_SRC
    src/panda_body.cpp
    ../utils/src/franka_adapter.cpp
    src/qbmove_communications.cpp
    src/softhand2.cpp
    )

set(${MODULE_NAME}_HEADER_FILES
    include/${PROJECT_NAME}/${MODULE_NAME}/panda_body.hpp
    include/${PROJECT_NAME}/softhand2/commands.h
    include/${PROJECT_NAME}/softhand2/qbmove_communications.h
    include/${PROJECT_NAME}/softhand2/softhand2.hpp
    )

add_library(${MODULE_NAME}
    ${${MODULE_NAME}_SRC}
    ${${MODULE_NAME}_HEADER_FILES}
    )


add_library(${PROJECT_NAME}::${MODULE_NAME} ALIAS ${MODULE_NAME})

find_package (Python3 COMPONENTS Interpreter Development)
find_package(PythonLibsNew REQUIRED)

target_include_directories(${MODULE_NAME}
    PRIVATE
    src
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${PYTHON_INCLUDE_DIRS}
    )

find_package(conv_vel2pose REQUIRED)

target_link_libraries(${MODULE_NAME}
    PRIVATE
    ${PROJECT_NAME}::${PROJECT_NAME}_project_options
    ${PROJECT_NAME}::${PROJECT_NAME}_project_warnings
    ${PROJECT_NAME}::${PROJECT_NAME}_project_deps
    ${PROJECT_NAME}::memory
    mirmi_cpp_utils::network
    mirmi_cpp_utils::conversion
    CONAN_PKG::pybind11
    CONAN_PKG::nlohmann_json
    CONAN_PKG::pybind11_json
    conv_vel2pose::conv_vel2pose
    PUBLIC
    ${PROJECT_NAME}::utils
    Franka::Franka
    Python3::Python
    )

apply_module_options(${MODULE_NAME})
