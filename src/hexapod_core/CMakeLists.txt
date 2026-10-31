cmake_minimum_required(VERSION 3.20)
project(hexapod_core)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

add_library(${PROJECT_NAME}
    src/kinematics/analytical_leg_kinematics.cpp
    src/actuator/joint.cpp
    src/robot/leg.cpp
    src/robot/hexapod_geometry.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

find_package(ament_cmake QUIET)

if(ament_cmake_FOUND)
    ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
    ament_export_include_directories(include)
    ament_export_libraries(${PROJECT_NAME})

    install(DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
    )

    install(TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include/${PROJECT_NAME}
    )

    if(BUILD_TESTING)
        find_package(ament_cmake_gtest REQUIRED)
        find_package(ament_lint_auto REQUIRED)
        set(ament_cmake_copyright_FOUND TRUE)
        set(ament_cmake_cpplint_FOUND TRUE)
        ament_lint_auto_find_test_dependencies()

        ament_add_gtest(test_analytical_leg_kinematics test/test_analytical_leg_kinematics.cpp)
        target_link_libraries(test_analytical_leg_kinematics ${PROJECT_NAME})

        ament_add_gtest(test_hexapod_geometry test/test_hexapod_geometry.cpp)
        target_link_libraries(test_hexapod_geometry ${PROJECT_NAME})

        ament_add_gtest(test_leg test/test_leg.cpp)
        target_link_libraries(test_leg ${PROJECT_NAME})
    endif()

    ament_package()
else()
    include(GNUInstallDirs)
    install(TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )
    install(DIRECTORY include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    )

    include(CTest)
    if(BUILD_TESTING)
        find_package(GTest REQUIRED)
        include(GoogleTest)

        add_executable(test_analytical_leg_kinematics test/test_analytical_leg_kinematics.cpp)
        target_link_libraries(test_analytical_leg_kinematics PRIVATE ${PROJECT_NAME} GTest::gtest_main)
        gtest_discover_tests(test_analytical_leg_kinematics)

        add_executable(test_hexapod_geometry test/test_hexapod_geometry.cpp)
        target_link_libraries(test_hexapod_geometry PRIVATE ${PROJECT_NAME} GTest::gtest_main)
        gtest_discover_tests(test_hexapod_geometry)

        add_executable(test_leg test/test_leg.cpp)
        target_link_libraries(test_leg PRIVATE ${PROJECT_NAME} GTest::gtest_main)
        gtest_discover_tests(test_leg)
    endif()
endif()
