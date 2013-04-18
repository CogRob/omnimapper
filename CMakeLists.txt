cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
#SET(CMAKE_BUILD_TYPE debug)
#SET(CMAKE_BUILD_TYPE distribution)
#SET(CMAKE_CXX_FLAGS_DISTRIBUTION "-O3")
#SET(CMAKE_C_FLAGS_DISTRIBUTION "-O3")
SET(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE)
SET(CMAKE_CXX_FLAGS_RELEASE "-O3")
SET(CMAKE_C_FLAGS_RELEASE "-O3")

#ROS
#include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#rosbuild_init()

project(omnimapper)

#add_definitions(-DUSE_ROS)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

include_directories(include)
#include_directories(/home/atrevor/git/omnimapper/omnimapper2_ros/include)
#link_directories(/home/atrevor/git/omnimapper/omnimapper2_ros/lib)

add_library(lib_omnimapper include/omnimapper/omnimapper_base.h
					     include/omnimapper/measurement_plugin.h 
					     include/omnimapper/pose_plugin.h
					     include/omnimapper/icp_pose_plugin.h
					     include/omnimapper/omnimapper_visualizer_pcl.h
					     #include/omnimapper/StampedSymbol.h
					     include/omnimapper/organized_feature_extraction.h
					     include/omnimapper/plane.h
					     include/omnimapper/plane_factor.h
					     include/omnimapper/transform_helpers.h
					     include/omnimapper/no_motion_pose_plugin.h
					     src/omnimapper_base.cpp
					     src/icp_pose_plugin.cpp
					     src/omnimapper_visualizer_pcl.cpp
					     src/organized_feature_extraction.cpp
					     src/plane.cpp
					     src/plane_factor.cpp
					     src/transform_helpers.cpp
					     src/no_motion_pose_plugin.cpp)
target_link_libraries(lib_omnimapper ${PCL_LIBRARIES} gtsam /opt/ros/groovy/lib/librostime.so)

#add_executable(omnimapper src/omnimapper.cpp)
#target_link_libraries(omnimapper lib_omnimapper)

add_executable(omnimapper_simple_icp_demo src/omnimapper_simple_icp_demo.cpp)
target_link_libraries(omnimapper_simple_icp_demo lib_omnimapper ${PCL_LIBRARIES} /opt/ros/groovy/lib/librostime.so)
#rosbuild_add_executable(omnimapper_simple_icp_demo src/omnimapper_simple_icp_demo.cpp)
#target_link_libraries(omnimapper_simple_icp_demo lib_omnimapper ${PCL_LIBRARIES})

add_executable(omnimapper_handheld src/omnimapper_handheld_demo.cpp)
target_link_libraries(omnimapper_handheld lib_omnimapper
${PCL_LIBRARIES} /opt/ros/groovy/lib/librostime.so)

add_executable(omnimapper_handheld_pcd src/omnimapper_handheld_pcd_demo.cpp)
target_link_libraries(omnimapper_handheld_pcd lib_omnimapper ${PCL_LIBRARIES} /opt/ros/groovy/lib/librostime.so)

#add_executable(isam_splice_test src/isam_splice_test.cpp)
#target_link_libraries(isam_splice_test gtsam)

add_executable(omnimapper_test src/omnimapper_test.cpp)
target_link_libraries(omnimapper_test lib_omnimapper /opt/ros/groovy/lib/librostime.so)
