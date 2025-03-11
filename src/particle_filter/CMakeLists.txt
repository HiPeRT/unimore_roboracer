cmake_minimum_required(VERSION 3.5.0)
project(particle_filter VERSION 1.0
        DESCRIPTION "particle_filter"
        LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -Wall -fopenmp")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -fopenmp)
endif()

# find_program(CLANGTIDY clang-tidy)
# if(CLANGTIDY)
#   set(CMAKE_CXX_CLANG_TIDY ${CLANGTIDY} -extra-arg=-Wno-unknown-warning-option)
#   if(${CMAKE_CXX_STANDARD})
#     set(CMAKE_CXX_CLANG_TIDY ${CMAKE_CXX_CLANG_TIDY} -extra-arg=-std=c++${CMAKE_CXX_STANDARD} --header-filter=${PROJECT_NAME})
#   endif()
# endif()

include(CheckLanguage)
check_language(CUDA)
if(CMAKE_CUDA_COMPILER)
    if(NOT DEFINED CMAKE_CUDA_STANDARD)
        set(CMAKE_CUDA_STANDARD ${CMAKE_CXX_STANDARD})
        set(CMAKE_CUDA_STANDARD_REQUIRED ON)
    endif()
    enable_language(CUDA)
else()
    message(STATUS "No CUDA support")
endif()

find_package(ament_cmake REQUIRED)

find_package(adx_data REQUIRED)
find_package(adx_data_ros REQUIRED)
find_package(adx_msgs REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(uio_map QUIET)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMP)

# uio_map library
if(uio_map_FOUND)
    add_library(pl_driver STATIC
        src/ray_casting/xrm.cpp
    )
    target_include_directories(pl_driver PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    )
    target_link_libraries(pl_driver PUBLIC
        uio_map::map
    )
    add_definitions(-DFPGA_SUPPORT_ENABLED)
    message(STATUS "FPGA drivers detected. Compiling FPGA driver support.")
else()
    message(STATUS "No FPGA drivers available.")
endif()


# ray_marching library
add_library(ray_marching
    src/ray_casting/ray_marching.cpp
    src/ray_casting/ray_casting.cpp
)
target_include_directories(ray_marching PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(ray_marching PUBLIC
    adx_data::particle
    adx_data::pose
    OpenMP::OpenMP_CXX
)

if(uio_map_FOUND)
    target_sources(ray_marching PRIVATE
        src/ray_casting/ray_marching_fpga.cpp
    )
    target_link_libraries(ray_marching PUBLIC
        pl_driver
    )
endif()

# if (CMAKE_CUDA_COMPILER)
#     add_library(
#         gpuRM STATIC
#         src/cudaRayMarching.cu
#     )
#     target_include_directories(gpuRM PUBLIC
#         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#         $<BUILD_INTERFACE:${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}>
#         $<INSTALL_INTERFACE:include>
#     )

#     target_compile_options(gpuRM PUBLIC -DTKCUDA_ENABLED)
#     set_target_properties(gpuRM
#         PROPERTIES
#             CUDA_SEPARABLE_COMPILATION ON
#             CUDA_RESOLVE_DEVICE_SYMBOLS ON
#     )

#     target_link_libraries(raymarching PUBLIC
#         gpuRM
#     )
# endif()

# particle_filter library
add_library(particle_filter
    src/particle_filter/particle_filter.cpp
)
target_include_directories(particle_filter PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(particle_filter PUBLIC
    ray_marching
    yaml-cpp
    ${OpenCV_LIBS}
)

# particle_filter_node executable
add_executable(particle_filter_node
    src/pf_main.cpp
)
target_include_directories(particle_filter_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(particle_filter_node PUBLIC
    rclcpp
    nav_msgs
    std_msgs
    sensor_msgs
    laser_geometry
    tf2
    tf2_ros
    tf2_geometry_msgs
    adx_data
    adx_data_ros
)

target_link_libraries(particle_filter_node PUBLIC
    particle_filter
)

# install targets
install(TARGETS
    ray_marching
    particle_filter
    particle_filter_node
    DESTINATION lib/${PROJECT_NAME}
)

# install files
install(DIRECTORY
    launch
    conf
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
