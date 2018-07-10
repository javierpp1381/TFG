cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(sift_z_keypoints_estimation)
set(CMAKE_CXX_FLAGS "-Wno-deprecated -Wno-psabi")
find_package(PCL 1.3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (sift_z_keypoints_estimation sift_z_keypoints_estimation.cpp)
target_link_libraries (sift_z_keypoints_estimation ${PCL_LIBRARIES})
