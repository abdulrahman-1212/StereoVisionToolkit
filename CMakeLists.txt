cmake_minimum_required(VERSION 3.14)
project(StereoVisionToolkit LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ─────────────────────────────────────────────────────────────────────────────
# Dependencies
# ─────────────────────────────────────────────────────────────────────────────
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# ─────────────────────────────────────────────────────────────────────────────
# Core Library
# ─────────────────────────────────────────────────────────────────────────────
# NOTE: If your .cpp files are directly in src/ instead of src/core/, 
# change the paths below to src/rectification.cpp and src/feature_matching.cpp
add_library(stereo_core STATIC
    src/core/rectification.cpp
    src/core/feature_matching.cpp
)

# FIX: Add the 'include' directory so that #include "core/..." resolves correctly
target_include_directories(stereo_core PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(stereo_core PUBLIC 
    ${OpenCV_LIBS} 
    Eigen3::Eigen
)

# Edge-device compatibility: std::filesystem requires explicit linking on GCC < 9
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9.0)
    target_link_libraries(stereo_core PUBLIC stdc++fs)
endif()

# ─────────────────────────────────────────────────────────────────────────────
# Test Executables
# ─────────────────────────────────────────────────────────────────────────────
add_executable(test_rectification tests/test_rectification.cpp)
target_link_libraries(test_rectification PRIVATE stereo_core)

add_executable(test_feature_matching tests/test_feature_matching.cpp)
target_link_libraries(test_feature_matching PRIVATE stereo_core)