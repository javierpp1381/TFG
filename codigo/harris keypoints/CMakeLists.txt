cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(harris_keypoints)

find_package(PCL 1.3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (harris_keypoints harris_keypoints.cpp)
target_link_libraries (harris_keypoints ${PCL_LIBRARIES})
