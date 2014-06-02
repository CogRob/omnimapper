cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(OMNIMAPPER)

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
include_directories(${GTSAM_INCLUDE_DIRS})
link_directories(${GTSAM_LIBRARY_DIRS})
message("GTSAM INCS: " ${GTSAM_EIGEN_INCLUDE_PREFIX})
message("GTSAM LIBS: " ${GTSAM_LIBRARY_DIRS})

###########################################################################
# Use GTSAM's Eigen
#set(EIGEN_INCLUDE_DIRS ${GTSAM_DIR}/../../../include/gtsam/3rdparty/Eigen/)
#message("EIGEN_INCLUDE_DIRS: " ${EIGEN_INCLUDE_DIRS})

# Build CogRob's Organized Segmentation Tools
add_subdirectory(organized_segmentation_tools_dev)

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

# OmniMapper Plugins
include_directories (plugins/include)

set (plugins_incs plugins/include/omnimapper/icp_plugin.h
				  plugins/include/omnimapper/no_motion_pose_plugin.h)

set (plugins_srcs plugins/src/icp_plugin.cpp
				  plugins/src/no_motion_pose_plugin.cpp)


set (incs ${base_incs} ${plugins_incs})
set (srcs ${base_srcs} ${plugins_srcs})

add_library (omnimapper SHARED ${incs} ${srcs})
target_link_libraries(omnimapper ${PCL_LIBRARIES} ${GTSAM_LIBRARIES} ${TBB_LIBRARIES})