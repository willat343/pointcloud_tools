enable_testing()

add_executable(${PROJECT_NAME}_tests
    main.cpp
    pclpointcloud_transform.cpp
    pclpointcloud_utilities.cpp
    pclpointcloud2_utilities.cpp
    src/test_instances.cpp
)

target_link_libraries(${PROJECT_NAME}_tests
    ${PROJECT_NAME}
    ${pointcloud_tools_SYSTEM_LIBRARIES}
    gtest_main
    gtest
    pthread
)

target_include_directories(${PROJECT_NAME}_tests PUBLIC
    ../include
    include
)

target_include_directories(${PROJECT_NAME}_tests SYSTEM PUBLIC
    ${pointcloud_tools_SYSTEM_INCLUDE_DIRS}
)

# Compile Options
target_compile_options(${PROJECT_NAME}_tests PRIVATE
    -Wall -Wextra #-Wpedantic
    #-Wno-gnu-anonymous-struct   # disables warnings about anonymous structs (needs >= g++-12 so disable -Wpedantic)
)

set_target_properties(${PROJECT_NAME}_tests
    PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON
)

# Ensure the tests link to the built library, not installed library (this works by setting BUILD_RPATH, and then
# stopping the linker from setting RUNPATH instead of RPATH (RPATH is checked before LD_LIBRARY_PATH, while
# RUNPATH is checked after))
target_link_options(${PROJECT_NAME}_tests PUBLIC
    -Wl,--disable-new-dtags
)

# Add test
add_test(NAME ${PROJECT_NAME}_tests COMMAND ${PROJECT_NAME}_tests)
