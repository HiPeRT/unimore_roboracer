cmake_minimum_required(VERSION 3.15)
project(adx_curve)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(adx_data REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(curve INTERFACE
)
add_library(${PROJECT_NAME}::curve ALIAS curve)
target_include_directories(curve INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(curve INTERFACE
    Eigen3::Eigen
)

add_library(spline STATIC
    src/spline.cpp
)
add_library(${PROJECT_NAME}::spline ALIAS spline)
target_include_directories(spline PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(spline PUBLIC
    adx_curve::curve
    adx_data::path
)

add_library(segment STATIC
    src/segment.cpp
)
add_library(${PROJECT_NAME}::segment ALIAS segment)
target_include_directories(segment PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(segment PUBLIC
    adx_curve::curve
)

install(TARGETS
    curve
    segment
    spline
    DESTINATION lib
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
