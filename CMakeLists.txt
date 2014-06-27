cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(Point_Cloud_Transformer)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (render src/render.cpp)
target_link_libraries (render ${PCL_LIBRARIES})


