cmake_minimum_required(VERSION 3.11)
project(pointcloud_tools_ros
    VERSION 0.3.0
    DESCRIPTION "Pointcloud Tools ROS"
    LANGUAGES CXX
)

# Find catkin and required components
find_package(catkin REQUIRED COMPONENTS
    message_filters
    message_generation
    pcl_conversions
    pcl_ros
    roscpp
    sensor_msgs
    tf2_ros

    pointcloud_tools_catkin
    statistics_msgs
)

# Find Eigen3
find_package(Eigen3 REQUIRED)

# Find PCL
find_package(PCL REQUIRED)

# Find convert
find_package(convert REQUIRED)

# Find pointcloud_tools
find_package(pointcloud_tools REQUIRED)

# Custom service files
add_service_files(
    DIRECTORY srv
    FILES
        file_to_message.srv
        message_to_file.srv
)

# Generate custom messages
generate_messages()

# Define catkin package
catkin_package(
    INCLUDE_DIRS
        include
    CATKIN_DEPENDS
        message_filters
        message_runtime
        pcl_conversions
        pcl_ros
        roscpp
        sensor_msgs
        tf2_ros

        pointcloud_tools_catkin
        statistics_msgs
    DEPENDS
        EIGEN3
        PCL
        convert
        pointcloud_tools
)

# File converter
add_executable(pointcloud_file_converter
    src/pointcloud_file_converter_main.cpp
    src/pointcloud_file_converter.cpp
)
add_dependencies(pointcloud_file_converter
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_generate_messages_cpp
)
target_include_directories(pointcloud_file_converter SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${pointcloud_tools_INCLUDE_DIRS}
)
target_include_directories(pointcloud_file_converter PUBLIC
    include
)
target_link_libraries(pointcloud_file_converter
    ${catkin_LIBRARIES}
    Eigen3::Eigen
    ${PCL_LIBRARIES}
    ${pointcloud_tools_LIBRARIES}
)
set_target_properties(pointcloud_file_converter
    PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON
)

# Analyser
add_executable(pointcloud_analyser
    src/pointcloud_analyser_main.cpp
    src/pointcloud_analyser.cpp
)
add_dependencies(pointcloud_analyser
    ${catkin_EXPORTED_TARGETS}
)
target_include_directories(pointcloud_analyser SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${pointcloud_tools_INCLUDE_DIRS}
)
target_include_directories(pointcloud_analyser PUBLIC
    include
)
target_link_libraries(pointcloud_analyser
    ${catkin_LIBRARIES}
    Eigen3::Eigen
    ${PCL_LIBRARIES}
    ${pointcloud_tools_LIBRARIES}
)
set_target_properties(pointcloud_analyser
    PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON
)

# Compute distances
add_executable(compute_distances
    src/compute_distances_main.cpp
)
add_dependencies(compute_distances
    ${catkin_EXPORTED_TARGETS}
)
target_include_directories(compute_distances SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${convert_INCLUDE_DIRS}
    ${pointcloud_tools_INCLUDE_DIRS}
)
target_include_directories(compute_distances PUBLIC
    include
)
target_link_libraries(compute_distances
    ${catkin_LIBRARIES}
    Eigen3::Eigen
    ${PCL_LIBRARIES}
    ${convert_LIBRARIES}
    ${pointcloud_tools_LIBRARIES}
)
target_compile_definitions(compute_distances PUBLIC
    ${convert_DEFINITIONS}
)
set_target_properties(compute_distances
    PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON
)
