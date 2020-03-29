cmake_minimum_required(VERSION 2.8.3)
project(omnimapper_ros)

# Load catkin and all dependencies required for this package
set(PKG_DEPENDS
  roscpp
  tf
  tf2
  visualization_msgs
  interactive_markers
  geometry_msgs
  laser_geometry
  nodelet
  pcl_conversions
  )

set(BUILD_DEPENDS cmake_modules ${PKG_DEPENDS})
set(RUN_DEPENDS ${PKG_DEPENDS})

set(omnimapper_DIR /usr/local/lib/omnimapper)

# Find catkin and required ROS packages
find_package(catkin REQUIRED COMPONENTS ${BUILD_DEPENDS})

# Find omnimapper
# find_package(omnimapper REQUIRED)

# Find GTSAM
find_package(GTSAM REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})

# Find Boost
find_package(Boost REQUIRED COMPONENTS system)

#Eigen
set(EIGEN_INCLUDE_DIRS ${GTSAM_DIR}/../../../include/gtsam/3rdparty/Eigen/)
message("EIGEN_INCLUDE_DIRS: " ${EIGEN_INCLUDE_DIRS})
include_directories(${EIGEN_INCLUDE_DIRS})

# PCL Library
find_package(PCL 1.7 REQUIRED)

# Link directories
link_directories(${PCL_LIBRARY_DIRS})

#set (CMAKE_CXX_STANDARD 11)

SET(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE)
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -std=c++11 -fpermissive -w")
SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -std=c++11 -fpermissive -w")

SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -std=c++11 -Werror -Wall -Wextra -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unused-parameter")
SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -std=c++11 -Werror -Wall -Wextra -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unused-parameter")
# Enable asan
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")
set(CMAKE_LINKER_FLAGS_DEBUG "${CMAKE_LINKER_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

message(------------------------)
get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY LINK_DIRECTORIES)
foreach(dir ${dirs})
  message(STATUS "link dir='${dir}'")
endforeach()

# # Build ROS Messages and Services
# add_message_files(FILES
#   ObjectInfo.msg
#   ObjectInfos.msg
# )

add_service_files(
  FILES
  OutputMapTSDF.srv
  PublishModel.srv
  VisualizeFullCloud.srv
  WriteTrajectoryFile.srv
  )

generate_messages(DEPENDENCIES
  geometry_msgs
  std_msgs
  sensor_msgs
  )

catkin_package(
  DEPENDS ${RUN_DEPENDS}
  INCLUDE_DIRS  include
  #LIBRARIES ${PROJECT_NAME}
  )

# Set include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  )


add_library(${PROJECT_NAME}
  src/ros_time_utils.cpp
  src/ros_tf_utils.cpp
  src/tf_pose_plugin.cpp
  src/omnimapper_visualizer_rviz.cpp
  src/tum_data_error_plugin.cpp
  src/get_transform_functor_tf.cpp
  src/error_evaluation_plugin.cpp
  src/canonical_scan_matcher_plugin.cpp
  src/canonical_scan.cpp
  src/csm_math_functions.cpp
  src/csm_visualizer.cpp
  src/omnimapper_ros.cpp
  src/ar_marker_plugin.cpp
  src/ros_log_sink.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES}
  omnimapper gtsam glog profiler tbb tbbmalloc csm)

add_executable(time_test src/time_test.cpp)
target_link_libraries(time_test ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(omnimapper_ros_nodelet src/omnimapper_ros_nodelet.cpp)
target_link_libraries(
  omnimapper_ros_nodelet ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} omnimapper)

add_executable(omnimapper_ros_node src/omnimapper_ros_node.cpp)
target_link_libraries(omnimapper_ros_node ${PROJECT_NAME} ${PCL_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} omnimapper gtsam glog tbbmalloc profiler)

add_executable(benchmark_icp_seq src/benchmark_icp_seq.cpp)
target_link_libraries(benchmark_icp_seq ${PROJECT_NAME} ${PCL_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} omnimapper gtsam glog tbbmalloc profiler tbb)

add_executable(benchmark_icp_lc src/benchmark_icp_lc.cpp)
target_link_libraries(benchmark_icp_lc ${PROJECT_NAME} ${PCL_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} omnimapper gtsam glog tbbmalloc profiler tbb)

add_executable(omnimapper_pcd src/omnimapper_ros_pcd.cpp)
target_link_libraries(omnimapper_pcd ${PROJECT_NAME} ${PCL_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} omnimapper gtsam glog tbbmalloc tbb)

add_executable(omnimapper_openni src/omnimapper_ros_openni.cpp)
target_link_libraries(omnimapper_openni ${PROJECT_NAME} ${PCL_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} omnimapper gtsam glog tbbmalloc tbb)
