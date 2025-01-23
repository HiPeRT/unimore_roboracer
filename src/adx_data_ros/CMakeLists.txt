cmake_minimum_required(VERSION 3.15)
project(adx_data_ros)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CXX clang++)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(adx_data REQUIRED)
find_package(adx_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

if(BUILD_TESTING)
    find_package(GTest REQUIRED)
endif()

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} INTERFACE
    Eigen3::Eigen
    adx_data::covariance
    adx_data::header
    adx_data::obstacle
    adx_data::odometry
    adx_data::particle
    adx_data::path
    adx_data::point
    adx_data::pose
    adx_data::quaternion
    adx_data::timestamp
    adx_data::twist
    adx_data::vector
)

ament_target_dependencies(${PROJECT_NAME} INTERFACE
    rclcpp
    std_msgs
    nav_msgs
    geometry_msgs
    adx_msgs
)

install(TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    LIBRARY DESTINATION lib
    INCLUDES DESTINATION include
)

install(DIRECTORY
    include/
    DESTINATION include
)

ament_export_dependencies()
ament_export_targets(export_${PROJECT_NAME})

if (GTest_FOUND)
    if(BUILD_TESTING)
        message(STATUS "Building tests")

        # Test Header
        add_executable(test_header
            test/test_header.cpp
        )
        include_directories(test_header "test")
        target_link_libraries(test_header PRIVATE
            adx_data_ros
            adx_data::header
            GTest::gtest_main
        )
        ament_target_dependencies(test_header PUBLIC
            tf2
            rclcpp
            std_msgs
        )
        add_test(NAME test_header
            COMMAND test_header
        )


        # Test Odometry
        add_executable(test_odometry
            test/test_odometry.cpp
        )
        include_directories(test_odometry "test")
        target_link_libraries(test_odometry PRIVATE
            adx_data_ros
            adx_data::odometry
            GTest::gtest_main
        )
        ament_target_dependencies(test_odometry PUBLIC
            rclcpp
            std_msgs
            tf2
        )
        add_test(NAME test_odometry
        COMMAND test_odometry
        )


        # Test Point
        add_executable(test_point
            test/test_point.cpp
        )
        target_include_directories(test_point PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
        include_directories(test_point "test")
        target_link_libraries(test_point PRIVATE
            adx_data_ros
            adx_data::point
            GTest::gtest_main
        )
        ament_target_dependencies(test_point PUBLIC
            rclcpp
            std_msgs
        )
        add_test(NAME test_point
        COMMAND test_point
        )


        # Test Pose
        add_executable(test_pose
            test/test_pose.cpp
        )
        target_include_directories(test_pose PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
        include_directories(test_pose "test")
        target_link_libraries(test_pose PRIVATE
            adx_data_ros
            adx_data::pose
            GTest::gtest_main
        )
        ament_target_dependencies(test_pose PUBLIC
            tf2
            geometry_msgs
            rclcpp
        )
        add_test(NAME test_pose
        COMMAND test_pose
        )


        # Test Quaternion
        add_executable(test_quaternion
            test/test_quaternion.cpp
        )
        target_include_directories(test_quaternion PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
        include_directories(test_quaternion "test")
        target_link_libraries(test_quaternion PRIVATE
            adx_data_ros
            adx_data::quaternion
            GTest::gtest_main
        )
        ament_target_dependencies(test_quaternion PUBLIC
            tf2
        )
        add_test(NAME test_quaternion
        COMMAND test_quaternion
        )


        # Test Twist
        add_executable(test_twist
            test/test_twist.cpp
        )
        target_include_directories(test_twist PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
        include_directories(test_twist "test")
        target_link_libraries(test_twist PRIVATE
            adx_data_ros
            adx_data::twist
            GTest::gtest_main
        )
        ament_target_dependencies(test_twist PUBLIC
            rclcpp
            std_msgs
            tf2
        )
        add_test(NAME test_twist
        COMMAND test_twist
        )


        # Test Plan
        add_executable(test_plan
            test/test_plan.cpp
        )
        target_include_directories(test_plan PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
        include_directories(test_plan "test")
        target_link_libraries(test_plan PRIVATE
            adx_data_ros
            adx_data::point
            adx_data::path
            GTest::gtest_main
        )
        ament_target_dependencies(test_plan PUBLIC
            rclcpp
            std_msgs
            adx_msgs
            tf2
        )
        add_test(NAME test_plan
        COMMAND test_plan
        )

    endif()
endif()
ament_package()