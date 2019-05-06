cmake_minimum_required(VERSION 3.5)
project(bipartite_surface_normal)

set(CMAKE_CXX_COMPILER "/usr/bin/g++")
set(CMAKE_CXX_FLAGS "-O0 -std=gnu++11 -fopenmp -fexceptions")

include_directories(
  include
)

add_executable(bipartite_surface_normal 
                src/main.cpp
                src/ParameterManager.cpp
              )
              
find_package(PkgConfig REQUIRED)
find_package(PCL REQUIRED)
find_package(VTK REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
target_link_libraries(bipartite_surface_normal ${PCL_LIBRARIES})
include_directories(${VTK_INCLUDE_DIRS})
target_link_libraries(bipartite_surface_normal ${VTK_LIBRARIES})
