cmake_minimum_required(VERSION 2.4.6)

#PCL Library
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

SET(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE)
SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -std=c++11")
SET(CMAKE_C_FLAGS_RELEASE "-O3 -std=c++11")

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)



rosbuild_init()

include_directories(../../base/include ../../plugins/include ../../organized_segmentation_tools_dev/include)
link_directories(../../build ../../build/organized_segmentation_tools_dev)

#find_package (CPUTSDF REQUIRED)
#include_directories(${CPUTSDF_INCLUDE_DIRS})
#link_directories(${CPUTSDF_LIBRARY_DIRS})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

message(------------------------)
get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY LINK_DIRECTORIES)
foreach(dir ${dirs})
  message(STATUS "link dir='${dir}'")
endforeach()


#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()


rosbuild_add_library(${PROJECT_NAME} 
				     src/ros_time_utils.cpp 
				     src/ros_tf_utils.cpp 
				     src/tf_pose_plugin.cpp 
				     src/omnimapper_visualizer_rviz.cpp 
				     src/get_transform_functor_tf.cpp 
				     src/canonical_scan_matcher_plugin.cpp
				     src/canonical_scan.cpp
				     src/csm_math_functions.cpp 
				     src/csm_visualizer.cpp
				     src/omnimapper_ros.cpp)
target_link_libraries(${PROJECT_NAME} omnimapper gtsam ${PCL_LIBRARIES} organized_segmentation_tools tcmalloc profiler tbb csm)

rosbuild_add_executable(time_test src/time_test.cpp)

rosbuild_add_library(omnimapper_ros_nodelet src/omnimapper_ros_nodelet.cpp)
target_link_libraries(omnimapper_ros_nodelet ${PROJECT_NAME})

rosbuild_add_executable(omnimapper_ros_node src/omnimapper_ros_node.cpp)
target_link_libraries(omnimapper_ros_node ${PROJECT_NAME} ${PCL_LIBRARIES} omnimapper gtsam tcmalloc profiler) 

rosbuild_add_executable(omnimapper_pcd src/omnimapper_ros_pcd.cpp)
target_link_libraries(omnimapper_pcd ${PROJECT_NAME} ${PCL_LIBRARIES} omnimapper gtsam tcmalloc tbb)

rosbuild_add_executable(omnimapper_openni src/omnimapper_ros_openni.cpp)
target_link_libraries(omnimapper_openni ${PROJECT_NAME} ${PCL_LIBRARIES} omnimapper gtsam tcmalloc tbb)
