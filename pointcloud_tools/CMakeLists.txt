cmake_minimum_required(VERSION 3.11)
project(pointcloud_tools
    VERSION 0.2.0
    DESCRIPTION "Pointcloud Tools"
    LANGUAGES CXX
)

# Options
option(BUILD_TESTS "Build Tests" OFF)

# Find Eigen3
find_package(Eigen3 REQUIRED)

# Find PCL
find_package(PCL REQUIRED)

# Find mathbox
find_package(mathbox REQUIRED)

# Find convert
find_package(convert REQUIRED)

set(SYSTEM_INCLUDE_DIRS
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${mathbox_INCLUDE_DIRS}
    ${convert_INCLUDE_DIRS}
)

set(SYSTEM_LIBRARIES
    Eigen3::Eigen
    ${PCL_LIBRARIES}
    ${mathbox_LIBRARIES}
    ${convert_LIBRARIES}
)

# Installation directories
include(GNUInstallDirs)

# Library
add_library(${PROJECT_NAME} SHARED
    src/pclpointcloud2_utilities.cpp
    src/point_types.cpp
)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
    ${SYSTEM_INCLUDE_DIRS}
)
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(${PROJECT_NAME}
    ${SYSTEM_LIBRARIES}
)
target_compile_definitions(${PROJECT_NAME} PUBLIC
    ${convert_DEFINITIONS}
    PCL_NO_PRECOMPILE
)
set_target_properties(${PROJECT_NAME}
    PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON
)

# Configure package version and config files
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${PROJECT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

# Installation
install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}
    PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}"
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(
    EXPORT ${PROJECT_NAME}
    DESTINATION ${PROJECT_NAME}
    FILE ${PROJECT_NAME}-export.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)
install(
    FILES
        "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)
install(
    DIRECTORY include/${PROJECT_NAME}
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
    FILES_MATCHING
        PATTERN "*.hpp"
        PATTERN "*.h"
)

# Uninstallation
if(NOT TARGET uninstall)
    configure_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in"
        "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
        @ONLY
    )

    # Add uninstall target
    add_custom_target(uninstall COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
endif()

# Testing
if(BUILD_TESTS)
    add_subdirectory(test)
endif()
