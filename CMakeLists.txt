cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(OMNIMAPPER)

set (CMAKE_CXX_STANDARD 11)

set (OMNIMAPPER_VERSION_MAJOR 0)
set (OMNIMAPPER_VERSION_MINOR 1)
set (OMNIMAPPER_VERSION_STRING "$OMNIMAPPER_VERSION_MAJOR}.${OMNIMAPPER_VERSION_MINOR}")

###########################################################################
# Find PCL
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

###########################################################################
# Find GTSAM
find_package(GTSAM REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})
link_directories(${GTSAM_LIBRARY_DIRS})
message("GTSAM INCS: " ${GTSAM_EIGEN_INCLUDE_PREFIX})
message("GTSAM LIBS: " ${GTSAM_LIBRARY_DIRS})

###########################################################################
# Use GTSAM's Eigen
#set(EIGEN_INCLUDE_DIRS ${GTSAM_DIR}/../../../include/gtsam/3rdparty/Eigen/)
#message("EIGEN_INCLUDE_DIRS: " ${EIGEN_INCLUDE_DIRS})

# Build CogRob's Organized Segmentation Tools
add_subdirectory(organized_segmentation_tools_dev)
include_directories(organized_segmentation_tools_dev/include)


# OmniMapper Base
include_directories (base/include)

set (base_incs base/include/omnimapper/omnimapper_base.h
			   base/include/omnimapper/output_plugin.h
			   base/include/omnimapper/pose_chain.h
			   base/include/omnimapper/pose_plugin.h
			   base/include/omnimapper/time.h
			   base/include/omnimapper/transform_tools.h
			   base/include/omnimapper/trigger.h
			   base/include/omnimapper/get_transform_functor.h)

set (base_srcs base/src/omnimapper_base.cpp
			   base/src/time.cpp
			   base/src/transform_tools.cpp)

# Landmarks for Object-SLAM
#include_directories(landmarks/include)

#set(landmarks_incs landmarks/include/omnimapper/object.h
#		   landmarks/include/omnimapper/landmark_factor.h)

#set(landmarks_srcs landmarks/src/object.cpp)

# OmniMapper Plugins
include_directories (plugins/include)

set (plugins_incs plugins/include/omnimapper/icp_plugin.h
		  #plugins/include/omnimapper/enp_plugin/enp_plugin.h
		  #plugins/include/omnimapper/enp_plugin/enp_base.h
		  #plugins/include/omnimapper/ebt_plugin/ebt_base.h
		  #plugins/include/omnimapper/ebt_plugin/ebt_plugin.h
		  #plugins/include/omnimapper/apriltag_plugin/apriltag_base.h
		  #plugins/include/omnimapper/apriltag_plugin/apriltag_plugin.h
		  plugins/include/omnimapper/no_motion_pose_plugin.h
		  #plugins/include/omnimapper/object_plugin.h
)

set (plugins_srcs plugins/src/icp_plugin.cpp
		  #plugins/src/enp_plugin.cpp
		  #plugins/src/ebt_plugin.cpp
		  #plugins/src/apriltag_plugin.cpp
		  plugins/src/no_motion_pose_plugin.cpp
		  #plugins/src/object_plugin.cpp
)


set (incs ${base_incs} ${landmarks_incs} ${plugins_incs})
set (srcs ${base_srcs} ${landmarks_srcs} ${plugins_srcs})

add_library (omnimapper SHARED ${incs} ${srcs})
target_link_libraries(omnimapper ${PCL_LIBRARIES} ${GTSAM_LIBRARIES} ${TBB_LIBRARIES})

###########################################################################
# INSTALL

# Install Omnimapper Headers
install(DIRECTORY ${CMAKE_SOURCE_DIR}/base/include/omnimapper DESTINATION include)

install(TARGETS omnimapper DESTINATION lib)

# Install Omnimapper Plugins
install(DIRECTORY ${CMAKE_SOURCE_DIR}/plugins/include/omnimapper DESTINATION include)

# Install Omnimapper organized segmentation tools
# install(DIRECTORY ${CMAKE_SOURCE_DIR}/organized_segmentation_tools_dev/include/organized_segmentation_tools DESTINATION include)

#install(EXPORT omnimapper DESTINATION lib/omnimapper)

# # Configure package config file
# configure_file(
#   ${CMAKE_SOURCE_DIR}/cmake/omnimapperConfig.cmake.in
#   ${CMAKE_BINARY_DIR}/cmake/omnimapperConfig.cmake @ONLY
# )
#
# set(ConfigPackageLocation lib/omnimapper)
#
# install(
#   FILES
#     ${CMAKE_BINARY_DIR}/cmake/omnimapperConfig.cmake
#   DESTINATION
#     ${ConfigPackageLocation})
