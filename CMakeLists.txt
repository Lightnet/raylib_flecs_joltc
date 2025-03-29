# Minimum CMake version
cmake_minimum_required(VERSION 3.14)

# Project name and languages
project(drop_cube LANGUAGES C CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Default build type to Debug if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

# Set output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

# Include FetchContent module for raylib
include(FetchContent)

# Declare raylib 5.5 from GitHub
FetchContent_Declare(
    raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG 5.5
)
FetchContent_MakeAvailable(raylib)

# Download CPM.cmake for dependency management
file(DOWNLOAD
    "https://github.com/cpm-cmake/CPM.cmake/releases/download/v0.40.7/CPM.cmake"
    "${CMAKE_BINARY_DIR}/CPM_0.40.7.cmake"
)
include(${CMAKE_BINARY_DIR}/CPM_0.40.7.cmake)

# Add joltc dependency (includes Jolt Physics internally)
CPMAddPackage(
    NAME joltc
    GITHUB_REPOSITORY amerkoleci/joltc
    GIT_TAG main
    OPTIONS
        "CMAKE_CXX_FLAGS /MDd"  # Ensure dynamic runtime
        "CMAKE_C_FLAGS /MDd"
)

# Add flecs dependency (v4.0.5)
CPMAddPackage(
    NAME flecs
    GITHUB_REPOSITORY SanderMertens/flecs
    GIT_TAG v4.0.5
)

# Create executable
add_executable(drop_cube
    src/main.c
)

# Link libraries (raylib, joltc, and flecs)
target_link_libraries(drop_cube PRIVATE
    raylib
    joltc
    flecs
)

# Include headers for raylib, joltc, and flecs
target_include_directories(drop_cube PRIVATE
    ${raylib_SOURCE_DIR}/src
    ${joltc_SOURCE_DIR}/include
    ${flecs_SOURCE_DIR}/include
)

# Platform-specific settings
if(WIN32)
    target_link_libraries(drop_cube PRIVATE ws2_32 winmm)
endif()