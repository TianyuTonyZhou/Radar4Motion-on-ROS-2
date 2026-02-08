cmake_minimum_required(VERSION 3.8)
project(radar4motion)

add_compile_options(-std=c++17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(angles REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(pcl_ros REQUIRED)

set(dependencies
    rclcpp
    sensor_msgs
    geometry_msgs
    std_msgs
    visualization_msgs
    nav_msgs
    tf2
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
    pcl_conversions
    angles
    PCL
    pcl_ros
)

option(USE_SYSTEM_EIGEN3 "Use system pre-installed Eigen" OFF)
option(USE_SYSTEM_SOPHUS "Use system pre-installed Sophus" OFF)
option(USE_SYSTEM_TSL-ROBIN-MAP "Use system pre-installed tsl_robin" OFF)
option(USE_SYSTEM_TBB "Use system pre-installed oneAPI/tbb" ON)
include(3rdparty/find_dependencies.cmake)

include_directories(
    include
    include/radar4motion
    include/radar4motion/ini_handler_cpp
    include/radar4motion/kiss-icp
    ${PCL_INCLUDE_DIRS}
    ${Open3D_INCLUDE_DIRS}
    ${OpenMP_CXX_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(${PROJECT_NAME}
    # main
    src/main.cpp
    # radar4motion
    src/core.cpp
    src/radar4motion/odometry.cpp
    src/radar4motion/preprocessing.cpp
    # kiss-icp
    include/radar4motion/kiss-icp/Preprocessing.cpp
    include/radar4motion/kiss-icp/Registration.cpp
    include/radar4motion/kiss-icp/VoxelHashMap.cpp
    include/radar4motion/kiss-icp/Threshold.cpp
    include/radar4motion/kiss-icp/VoxelUtils.cpp
    include/radar4motion/kiss-icp/Deskew.cpp
    # radar ego motion estimation
    include/ego_motion_estimator/ego_motion_estimator.cpp
    include/ego_motion_estimator/radar_point_cloud.cpp
    # Vod parse utils
    include/vod_utils/vod_parse_utils.cpp
)

# Apply ROS2 dependencies first (this uses plain signature internally)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

# Then add additional libraries using plain signature (to match ament_target_dependencies)
if(TARGET TBB::tbb)
    set(TBB_LIBS TBB::tbb)
else()
    set(TBB_LIBS "")
endif()

target_link_libraries(${PROJECT_NAME}
    ${LIBRARIES}
    ${PCL_LIBRARIES}
    Eigen3::Eigen
    tsl::robin_map
    Sophus::Sophus
    ${TBB_LIBS}
    ${OpenMP_CXX_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config dataset docs
    DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
    DESTINATION include
)

ament_package()