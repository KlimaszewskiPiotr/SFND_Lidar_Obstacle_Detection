cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_definitions(-std=c++14)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(playback)

find_package(PCL CONFIG REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")


add_executable (environment src/environment.cpp src/render/render.cpp src/processPointClouds.cpp src/ransac/ransac3d.cpp src/clusters/clusters.cpp)
add_executable(cluster src/quiz/cluster/cluster.cpp src/render/render.cpp src/processPointClouds.cpp)
#add_executable(ransac src/quiz/ransac/ransac2d.cpp src/render/render.cpp src/processPointClouds.cpp)
target_link_libraries (environment ${PCL_LIBRARIES})
target_link_libraries(cluster ${PCL_LIBRARIES})
#target_link_libraries (ransac ${PCL_LIBRARIES})





