cmake_minimum_required(VERSION 3.15)
project(adx_data)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Eigen3 REQUIRED)
find_package(GTest QUIET)

# Covariance
add_library(covariance INTERFACE)
add_library(${PROJECT_NAME}::covariance ALIAS covariance)
target_include_directories(covariance INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(covariance INTERFACE
    Eigen3::Eigen
)
if(Boost_FOUND)
    target_link_libraries(covariance INTERFACE
        Boost::boost
    )
    target_compile_options(covariance INTERFACE -DTKBOOST_ENABLED)
endif()

# TimeStamp
add_library(timestamp STATIC
    src/timestamp.cpp
)
add_library(${PROJECT_NAME}::timestamp ALIAS timestamp)
target_include_directories(timestamp PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(timestamp
    PUBLIC
        Eigen3::Eigen
)

# vector
add_library(vector INTERFACE)
add_library(${PROJECT_NAME}::vector ALIAS vector)
target_include_directories(vector INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(vector INTERFACE
    Eigen3::Eigen
)

# Header
add_library(header STATIC
    src/header.cpp
)
add_library(${PROJECT_NAME}::header ALIAS header)
target_include_directories(header PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(header PUBLIC
    adx_data::timestamp
    Eigen3::Eigen
)

# Point
add_library(point INTERFACE)
add_library(${PROJECT_NAME}::point ALIAS point)
target_include_directories(point INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(point INTERFACE
    adx_data::vector
)

# Quaternion
add_library(quaternion INTERFACE)
add_library(${PROJECT_NAME}::quaternion ALIAS quaternion)
target_include_directories(quaternion INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(quaternion INTERFACE
    Eigen3::Eigen
)

# Pose
add_library(pose INTERFACE)
add_library(${PROJECT_NAME}::pose ALIAS pose)
target_include_directories(pose INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(pose INTERFACE
    adx_data::header
    adx_data::covariance
    adx_data::point
    adx_data::quaternion
    adx_data::vector
)

# Particle
add_library(particle INTERFACE)
add_library(${PROJECT_NAME}::particle ALIAS particle)
target_include_directories(particle INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(particle INTERFACE
    adx_data::pose
)

# Twist
add_library(twist INTERFACE)
add_library(${PROJECT_NAME}::twist ALIAS twist)
target_include_directories(twist INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(twist INTERFACE
    adx_data::covariance
    adx_data::vector
)

# Path
add_library(path INTERFACE)
add_library(${PROJECT_NAME}::path ALIAS path)
target_include_directories(path INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(path INTERFACE
    adx_data::header
    adx_data::pose
    adx_data::vector
)

# Odometry
add_library(odometry STATIC
    src/odometry.cpp
)
add_library(${PROJECT_NAME}::odometry ALIAS odometry)
target_include_directories(odometry PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(odometry PUBLIC
    adx_data::covariance
    adx_data::header
    adx_data::pose
    adx_data::twist
)

# Obstacle
add_library(obstacle INTERFACE)
add_library(${PROJECT_NAME}::obstacle ALIAS obstacle)
target_include_directories(obstacle INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(obstacle INTERFACE
    adx_data::covariance
    adx_data::pose
    adx_data::twist
    adx_data::vector
)

# Install and export
install(TARGETS
    covariance
    header
    obstacle
    odometry
    path
    point
    pose
    particle
    quaternion
    timestamp
    twist
    vector
    EXPORT ${PROJECT_NAME}Targets
)

install(DIRECTORY
    include/
    DESTINATION include
)

install(EXPORT ${PROJECT_NAME}Targets
    NAMESPACE ${PROJECT_NAME}::
    FILE ${PROJECT_NAME}Targets.cmake
    DESTINATION share/${PROJECT_NAME}/cmake
)

include(CMakePackageConfigHelpers)
configure_package_config_file(Config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    INSTALL_DESTINATION share/${PROJECT_NAME}/cmake
    NO_SET_AND_CHECK_MACRO
    NO_CHECK_REQUIRED_COMPONENTS_MACRO
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    VERSION 1.0.0
    COMPATIBILITY AnyNewerVersion
)

export(EXPORT ${PROJECT_NAME}Targets
    NAMESPACE ${PROJECT_NAME}::
    FILE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    DESTINATION share/${PROJECT_NAME}/cmake
)

# Tests
if(GTest_FOUND)
    if(BUILD_TESTING)
        add_executable(test_covariance
            test/test_covariance.cpp
        )
        target_link_libraries(test_covariance PRIVATE
            adx_data::covariance
            GTest::gtest_main
        )
        add_test(NAME test_covariance
            COMMAND test_covariance
        )
    endif()
endif()