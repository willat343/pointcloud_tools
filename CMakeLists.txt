cmake_minimum_required(VERSION 3.5)
project(pointcloud_tools
    VERSION 1.0
    DESCRIPTION "Pointcloud Tools"
    LANGUAGES CXX
)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
    message_filters
    message_generation
    pcl_conversions
    pcl_ros
    roscpp
    sensor_msgs
    tf2_ros

    statistics_msgs
)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(convert REQUIRED)
find_package(mathbox REQUIRED)

add_service_files(
    DIRECTORY srv
    FILES
        file_to_message.srv
        message_to_file.srv
)

generate_messages(
    # DEPENDENCIES
)

set(LIBRARY ${PROJECT_NAME})

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

        statistics_msgs
    LIBRARIES ${LIBRARY}
    DEPENDS
        EIGEN3
        PCL
        convert
        mathbox
)

set(SYSTEM_INCLUDE_DIRS
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${convert_INCLUDE_DIRS}
)

set(INCLUDE_DIRS
    include
)

set(SYSTEM_LIBRARIES
    ${catkin_LIBRARIES}
    Eigen3::Eigen
    ${PCL_LIBRARIES}
    ${convert_LIBRARIES}
    mathbox
)

# Library
add_library(${LIBRARY}
    src/pclpointcloud2_utilities.cpp
    src/point_types.cpp
)
add_dependencies(${LIBRARY}
    ${catkin_EXPORTED_TARGETS}
)
target_include_directories(${LIBRARY} PUBLIC SYSTEM
    ${SYSTEM_INCLUDE_DIRS}
)
target_include_directories(${LIBRARY} PUBLIC
    ${INCLUDE_DIRS}
)
target_link_libraries(${LIBRARY}
    ${SYSTEM_LIBRARIES}
)
target_compile_definitions(${LIBRARY} PUBLIC
    ${convert_DEFINITIONS}
    PCL_NO_PRECOMPILE
)
set_target_properties(${LIBRARY}
    PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON
)

# File converter
add_executable(pointcloud_file_converter
    src/nodes/pointcloud_file_converter_main.cpp
    src/nodes/pointcloud_file_converter.cpp
)
add_dependencies(pointcloud_file_converter
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_generate_messages_cpp
)
target_include_directories(pointcloud_file_converter PUBLIC SYSTEM
    ${SYSTEM_INCLUDE_DIRS}
)
target_include_directories(pointcloud_file_converter PUBLIC
    ${INCLUDE_DIRS}
)
target_link_libraries(pointcloud_file_converter
    ${SYSTEM_LIBRARIES}
    ${LIBRARY}
)
set_target_properties(pointcloud_file_converter
    PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON
)

# Analyser
add_executable(pointcloud_analyser
    src/nodes/pointcloud_analyser_main.cpp
    src/nodes/pointcloud_analyser.cpp
)
add_dependencies(pointcloud_analyser
    ${catkin_EXPORTED_TARGETS}
)
target_include_directories(pointcloud_analyser PUBLIC SYSTEM
    ${SYSTEM_INCLUDE_DIRS}
)
target_include_directories(pointcloud_analyser PUBLIC
    ${INCLUDE_DIRS}
)
target_link_libraries(pointcloud_analyser
    ${SYSTEM_LIBRARIES}
    ${LIBRARY}
)
set_target_properties(pointcloud_analyser
    PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON
)

# Compute distances
add_executable(compute_distances
    src/nodes/compute_distances_main.cpp
)
add_dependencies(compute_distances
    ${catkin_EXPORTED_TARGETS}
)
target_include_directories(compute_distances PUBLIC SYSTEM
    ${SYSTEM_INCLUDE_DIRS}
)
target_include_directories(compute_distances PUBLIC
    ${INCLUDE_DIRS}
    ${convert_INCLUDE_DIRS}
)
target_link_libraries(compute_distances
    ${SYSTEM_LIBRARIES}
    ${LIBRARY}
)
target_compile_definitions(${LIBRARY} PUBLIC
    ${convert_DEFINITIONS}
)
set_target_properties(compute_distances
    PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON
)

# Testing
if(CATKIN_ENABLE_TESTING)
    catkin_add_gtest(test_${PROJECT_NAME}
        test/main.cpp
        test/pclpointcloud_transform.cpp
        test/pclpointcloud_utilities.cpp
        test/pclpointcloud2_utilities.cpp
        test/src/test_instances.cpp
    )
    target_include_directories(test_${PROJECT_NAME} SYSTEM PUBLIC
        ${SYSTEM_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
    )
    target_include_directories(test_${PROJECT_NAME} PUBLIC
        include
        test/include
    )
    target_link_libraries(test_${PROJECT_NAME}
        ${SYSTEM_LIBRARIES}
        ${LIBRARY}
    )
    set_target_properties(test_${PROJECT_NAME}
        PROPERTIES
            CXX_STANDARD 17
            CXX_STANDARD_REQUIRED ON
    )
endif(CATKIN_ENABLE_TESTING)
