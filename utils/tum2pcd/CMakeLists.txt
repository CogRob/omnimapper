cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
#SET(CMAKE_BUILD_TYPE debug)
#SET(CMAKE_BUILD_TYPE distribution)
SET(CMAKE_BUILD_TYPE Release)
SET(CMAKE_CXX_FLAGS_DISTRIBUTION "-O3")
SET(CMAKE_C_FLAGS_DISTRIBUTION "-O3")

SET(BOOST_INCLUDE_DIRS /usr/local/include)
SET(BOOST_LIBRARY_DIRS /usr/local/lib)

set(USE_GICP 1)

project(mapping)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})
add_definitions("-DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET -DFLANN_STATIC -DEIGEN_USE_NEW_STDVECTOR" )

include_directories(build)
include_directories(include)

#############################
# BOOST #####################
#############################
#find_package(BOOST)
include_directories(${BOOST_INCLUDE_DIRS})
link_directories(${BOOST_LIBRARY_DIRS})
#add_definitions(${BOOST_DEFINITIONS})

#############################
# OpenCV ####################
#############################
find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

#############################
# GICP ######################
#############################


# My edges
#include_directories(${CMAKE_SOURCE_DIR}/src/new_rgbd_edges/)
#SET(PCL_MY_EDGE src/new_rgbd_edges/organized_edge_detection.hpp src/new_rgbd_edges/organized_edge_detection.cpp)


SET(CMAKE_CXX_FLAGS -fpermissive)

add_executable(tum2pcd src/tum2pcd.cpp)
target_link_libraries(tum2pcd ${PCL_LIBRARIES} ${BOOST_LIBRARIES} ${OpenCV_LIBS})


