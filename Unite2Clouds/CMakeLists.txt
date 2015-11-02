cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(unite_2_clouds)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (unite_2_clouds unite_2_clouds.cpp)
target_link_libraries (unite_2_clouds ${PCL_LIBRARIES})