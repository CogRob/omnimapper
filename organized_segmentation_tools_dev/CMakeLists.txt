cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(organized_segmentation_tools)

SET(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE)
SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -std=c++11")
SET(CMAKE_C_FLAGS_RELEASE "-O3 -std=c++11")

#set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
message (MODULE_PATH= ${CMAKE_MODULE_PATH})

###########################################################################
# Find PCL
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

###########################################################################
# Find GTSAM
#find_package(GTSAM REQUIRED)
#include_directories(${GTSAM_INCLUDE_DIR})

###########################################################################
# Use GTSAM's Eigen
# Use generic Eigen include paths e.g. <Eigen/Core>
#set(EIGEN_INCLUDE_DIRS ${GTSAM_DIR}/../../../include/gtsam/3rdparty/Eigen/)
#message("EIGEN_INCLUDE_DIRS: " ${EIGEN_INCLUDE_DIRS})    

include_directories(include)
set(incs include/organized_segmentation_tools/organized_segmentation_tbb.h)

set(srcs src/organized_segmentation_tbb.cpp)

add_library(organized_segmentation_tools SHARED ${incs} ${srcs})
target_link_libraries(organized_segmentation_tools ${PCL_LIBRARIES} tbb tbbmalloc)

add_executable(organized_segmentation_tbb_demo src/organized_segmentation_tbb_demo.cpp)
target_link_libraries(organized_segmentation_tbb_demo organized_segmentation_tools)
