cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(omnimapper)

SET(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE)

set(IGNORE_WARNING_FLAGS "-Wno-unused-variable -Wno-unused-but-set-variable -Wno-unused-parameter -Wno-deprecated-declarations")
SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -std=c++11 -Werror -Wall -Wextra ${IGNORE_WARNING_FLAGS}")
SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -std=c++11 -Werror -Wall -Wextra ${IGNORE_WARNING_FLAGS}")
# Enable asan
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")
set(CMAKE_LINKER_FLAGS_DEBUG "${CMAKE_LINKER_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")

SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -std=c++11 ${IGNORE_WARNING_FLAGS}")
SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -std=c++11 ${IGNORE_WARNING_FLAGS}")

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
message (MODULE_PATH= ${CMAKE_MODULE_PATH})

###########################################################################
# Find PCL
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

###########################################################################
# Find GTSAM
# find_package(GTSAM REQUIRED)
# include_directories(${GTSAM_INCLUDE_DIR})
# Use GTSAM's Eigen
message("EIGEN_INCLUDE_DIRS: " ${EIGEN_INCLUDE_DIRS})
message("GTSAM_LIBS: " ${GTSAM_LIBS})

set(GTSAM_LINK_LIB "gtsam")

###########################################################################
# Find MKL
find_package(MKL REQUIRED)
include_directories(${MKL_INCLUDE_DIR})

###########################################################################
# Find CPUTSDF
set(CPUTSDF_DIR "/usr/local/share/cpu_tsdf")
find_package(CPUTSDF REQUIRED)
include_directories(${CPUTSDF_INCLUDE_DIRS})
link_directories(${CPUTSDF_LIBRARY_DIRS})

###########################################################################

include_directories(include)

set(incs
  include/omnimapper/geometry.h
  include/omnimapper/get_transform_functor.h
  include/omnimapper/icp_pose_plugin.h
  include/omnimapper/landmark_factor.h
  include/omnimapper/measurement_plugin.h
  include/omnimapper/no_motion_pose_plugin.h
  include/omnimapper/object_discovery.h
  include/omnimapper/object.h
  include/omnimapper/object_plugin.h
  include/omnimapper/object_recognition.h
  include/omnimapper/object_segment_propagation.h
  include/omnimapper/omnimapper_base.h
  include/omnimapper/omnimapper_visualizer_pcl.h
  include/omnimapper/organized_feature_extraction.h
  include/omnimapper/organized_feature_extraction_tbb.h
  include/omnimapper/output_plugin.h
  include/omnimapper/plane_factor.h
  include/omnimapper/plane.h
  include/omnimapper/plane_plugin.h
  include/omnimapper/pose_chain.h
  include/omnimapper/pose_plugin.h
  include/omnimapper/time.h
  include/omnimapper/transform_helpers.h
  include/omnimapper/tsdf_output_plugin.h
  include/omnimapper/BoundedPlane3.h
  include/omnimapper/BoundedPlaneFactor.h
  include/omnimapper/bounded_plane_plugin.h
  include/omnimapper/impl/geometry.hpp
)

set(third_party_incs
  include/omnimapper/3rdparty/distortion_model_standalone.h
  include/omnimapper/3rdparty/eigen_extensions.h
)

set(srcs
  src/omnimapper_base.cpp
  src/time.cpp
  src/icp_pose_plugin.cpp
  src/omnimapper_visualizer_pcl.cpp
  src/organized_feature_extraction.cpp
  src/organized_feature_extraction_tbb.cpp
  src/plane.cpp
  src/object.cpp
  src/plane_factor.cpp
  src/plane_plugin.cpp
  src/object_plugin.cpp
  src/object_segment_propagation.cpp
  src/object_recognition.cpp
  src/object_discovery.cpp
  src/transform_helpers.cpp
  src/no_motion_pose_plugin.cpp
  src/tsdf_output_plugin.cpp
  src/BoundedPlane3.cpp
  src/BoundedPlaneFactor.cpp
  src/bounded_plane_plugin.cpp
)

add_library(omnimapper SHARED ${incs} ${srcs})
target_link_libraries(omnimapper
  ${GTSAM_LINK_LIB} ${MKL_LIBRARIES} ${PCL_LIBRARIES}
  tbb tbbmalloc cpu_tsdf profiler)

install(TARGETS omnimapper DESTINATION lib)
install(FILES ${incs} DESTINATION include/${PROJECT_NAME})
install(FILES ${third_party_incs} DESTINATION include/${PROJECT_NAME}/3rdparty)

configure_file(${PROJECT_SOURCE_DIR}/cmake/omnimapperConfig.cmake.in
               ${CMAKE_CURRENT_BINARY_DIR}/omnimapperConfig.cmake)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/omnimapperConfig.cmake
        DESTINATION share/omnimapper)

add_executable(omnimapper_test src/omnimapper_test.cpp)
target_link_libraries(omnimapper_test omnimapper)

add_executable(omnimapper_static_test src/omnimapper_test.cpp ${srcs})
target_link_libraries(omnimapper_static_test
  ${GTSAM_LINK_LIB} ${MKL_LIBRARIES} ${PCL_LIBRARIES}
  tbb tbbmalloc cpu_tsdf profiler)

# add_executable(omnimapper_handheld src/omnimapper_handheld_demo.cpp)
# target_link_libraries(omnimapper_handheld omnimapper ${PCL_LIBRARIES} tbb)

# add_executable(omnimapper_plane_test src/omnimapper_plane_test.cpp)
# target_link_libraries(omnimapper_plane_test omnimapper ${PCL_LIBRARIES} tbb)
#
# add_executable(organized_feature_extraction_demo src/organized_feature_extraction_demo.cpp)
# target_link_libraries(organized_feature_extraction_demo
#   omnimapper ${PCL_LIBRARIES} /usr/lib/x86_64-linux-gnu/libX11.so.6 tbb)
#
# add_executable(organized_feature_extraction_demo_tbb src/organized_feature_extraction_demo_tbb.cpp)
# target_link_libraries(organized_feature_extraction_demo_tbb
#   omnimapper ${PCL_LIBRARIES} /usr/lib/x86_64-linux-gnu/libX11.so.6 tbb)
#
# add_executable(tbb_test src/tbb_test.cpp)
# target_link_libraries(tbb_test omnimapper ${PCL_LIBRARIES})

# add_executable(omnimapper src/omnimapper.cpp)
# target_link_libraries(omnimapper omnimapper)

# add_executable(omnimapper_simple_icp_demo src/omnimapper_simple_icp_demo.cpp)
# target_link_libraries(omnimapper_simple_icp_demo omnimapper ${PCL_LIBRARIES})

# add_executable(omnimapper_handheld_pcd src/omnimapper_handheld_pcd_demo.cpp)
# target_link_libraries(omnimapper_handheld_pcd omnimapper
# ${PCL_LIBRARIES} /opt/ros/groovy/lib/librostime.so)

# add_executable(isam_splice_test src/isam_splice_test.cpp)
# target_link_libraries(isam_splice_test gtsam)

# add_executable(omnimapper_plane_test2 src/omnimapper_plane_test2.cpp)
# target_link_libraries(omnimapper_plane_test2 omnimapper ${PCL_LIBRARIES})

# add_executable(tbb_test2 src/tbb_test2.cpp)
# target_link_libraries(tbb_test2 omnimapper ${PCL_LIBRARIES} tbb)
