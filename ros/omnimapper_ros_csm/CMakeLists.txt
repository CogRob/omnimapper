cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)


#PCL Library
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

rosbuild_init()
include_directories(../../include)
link_directories(../../build)



#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_library(${PROJECT_NAME}
src/canonical_scan_matcher_plugin.cpp src/canonical_scan.cpp src/math_functions.cpp src/csm_visualizer.cpp)
target_link_libraries(${PROJECT_NAME} omnimapper gtsam ${PCL_LIBRARIES})

rosbuild_add_executable(omnimapper_pelican src/omnimapper_ros_pelican.cpp)
target_link_libraries(omnimapper_pelican omnimapper omnimapper_ros omnimapper_ros_csm gtsam ${PCL_LIBRARIES})
