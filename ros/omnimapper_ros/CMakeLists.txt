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
link_directories(${PCL_LIBRARY_DIRS})

rosbuild_init()
include_directories(../../include)
link_directories(../../build)

include_directories(../../3rdparty/distortion_model/include)

include_directories(/home/siddharth/Research/projects/workspace/cpu_tsdf/include)
link_directories(/home/siddharth/Research/projects/workspace/cpu_tsdf/build)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
rosbuild_add_library(${PROJECT_NAME} src/ros_time_utils.cpp src/ros_tf_utils.cpp src/tf_pose_plugin.cpp src/omnimapper_visualizer_rviz.cpp src/tum_data_error_plugin.cpp src/get_transform_functor_tf.cpp src/error_evaluation_plugin.cpp)
target_link_libraries(${PROJECT_NAME} omnimapper gtsam ${PCL_LIBRARIES})

rosbuild_add_executable(time_test src/time_test.cpp)

#rosbuild_add_executable(omnimapper_ros_handheld src/omnimapper_ros_handheld.cpp)
#target_link_libraries(omnimapper_ros_handheld ${PROJECT_NAME} ${PCL_LIBRARIES} omnimapper gtsam)

rosbuild_add_executable(omnimapper_ros_node src/omnimapper_ros_node.cpp)
target_link_libraries(omnimapper_ros_node ${PROJECT_NAME} ${PCL_LIBRARIES} omnimapper gtsam) 
