cmake_minimum_required(VERSION 3.5)
project(adx_msgs)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(shape_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    msg/ArbitratedAckermannDrive.msg
    msg/LapTime.msg
    msg/Obstacle.msg
    msg/ObstacleArray.msg
    msg/PlanPoint.msg
    msg/Plan.msg
    msg/SectorStamped.msg
    msg/SectorSet.msg
    msg/Track.msg
    msg/TrackArray.msg
    DEPENDENCIES
        builtin_interfaces
        ackermann_msgs
        std_msgs
        geometry_msgs
        nav_msgs
        shape_msgs
)

ament_package()