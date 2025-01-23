cmake_minimum_required(VERSION 3.5)
project(purepursuit)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(adx_msgs REQUIRED)

# Other internal dependencies
find_package(adx_data REQUIRED)
find_package(adx_data_ros REQUIRED)
find_package(adx_curve REQUIRED)

# External Dependencies
find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME} STATIC
    src/${PROJECT_NAME}/${PROJECT_NAME}.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}
    adx_curve::curve
    Eigen3::Eigen
)

add_executable(${PROJECT_NAME}_node
    src/${PROJECT_NAME}_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node PRIVATE
    ${PROJECT_NAME}
    adx_data::path
    adx_curve::spline
    yaml-cpp
)

ament_target_dependencies(${PROJECT_NAME}_node PUBLIC
    ackermann_msgs
    adx_msgs
    geometry_msgs
    nav_msgs
    std_msgs
    adx_data_ros
)

install(TARGETS
    ${PROJECT_NAME}_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
    conf
    DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}/
)
ament_package()