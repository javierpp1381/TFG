cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(sift_keypoints)

find_package(PCL 1.3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (sift_keypoints sift_keypoints.cpp)
target_link_libraries (sift_keypoints ${PCL_LIBRARIES})
