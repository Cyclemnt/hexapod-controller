cmake_minimum_required(VERSION 3.20)
project(hexapod_controller)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(hexapod_core REQUIRED)

add_library(${PROJECT_NAME}_lib
    src/joint_mapping.cpp
    src/standing_controller_node.cpp
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_lib
    rclcpp
    geometry_msgs
    std_msgs
    sensor_msgs
    hexapod_core
)

add_executable(standing_controller_node
    src/standing_controller_main.cpp
)

target_link_libraries(standing_controller_node
    ${PROJECT_NAME}_lib
)

install(TARGETS
    ${PROJECT_NAME}_lib
    standing_controller_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
    DESTINATION include
)

install(DIRECTORY config launch
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    find_package(ament_index_cpp REQUIRED)
    find_package(urdf REQUIRED)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    set(ament_cmake_uncrustify_FOUND TRUE)
    set(ament_cmake_flake8_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()

    ament_add_gtest(test_urdf_geometry_sync test/test_urdf_geometry_sync.cpp)
    target_link_libraries(test_urdf_geometry_sync hexapod_core::hexapod_core)
    ament_target_dependencies(test_urdf_geometry_sync ament_index_cpp urdf)
endif()

ament_package()
