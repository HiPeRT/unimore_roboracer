cmake_minimum_required(VERSION 3.5)
project(path_publisher)

set(CMAKE_CXX_STANDARD 17 CACHE STRING "The C++ standard to use")
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(adx_data REQUIRED)
find_package(adx_data_ros REQUIRED)

find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(adx_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(${PROJECT_NAME} STATIC
    src/${PROJECT_NAME}.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
    adx_data::point
    adx_data::path
)

add_executable(${PROJECT_NAME}_node
    src/${PROJECT_NAME}_node.cpp
    src/main.cpp
)
target_link_libraries(${PROJECT_NAME}_node PRIVATE
    ${PROJECT_NAME}
)

ament_target_dependencies(${PROJECT_NAME}_node PUBLIC
    rclcpp
    adx_msgs
    geometry_msgs
    nav_msgs
    adx_data_ros
)

install(TARGETS
    ${PROJECT_NAME}_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
)

ament_package()