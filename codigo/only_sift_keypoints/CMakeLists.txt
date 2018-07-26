cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(only_sift_keypoints)

find_package(PCL 1.3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (only_sift_keypoints only_sift_keypoints.cpp)
target_link_libraries (only_sift_keypoints ${PCL_LIBRARIES})
