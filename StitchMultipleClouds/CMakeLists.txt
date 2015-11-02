cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(stitch_multiple_clouds)

find_package(PCL 1.5 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (stitch_multiple_clouds stitch_multiple_clouds.cpp)
target_link_libraries (stitch_multiple_clouds ${PCL_LIBRARIES})