cmake_minimum_required(VERSION 3.5)
project(omnimapper)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS timer thread)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(PCL 1.8 REQUIRED)

include_directories(
  include
)

include_directories(
  ${Boost_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${PCL_COMMON_INCLUDE_DIRS}
  ${OPENNI_INCLUDE_DIRS}
)

set (library_srcs
  src/omnimapper_base.cpp
  src/time.cpp
  src/transform_tools.cpp
  src/BoundedPlane3.cpp
  src/BoundedPlaneFactor.cpp
  src/plane_factor.cpp
  src/plane.cpp
  src/omnimapper_visualizer_pcl
)

set (plugins_srcs
  src/plugins/icp_plugin.cpp
  src/plugins/no_motion_pose_plugin.cpp
  src/plugins/bounded_plane_plugin.cpp
  src/plugins/plane_plugin.cpp
)

set (organized_segmentation_srcs
  src/organized_segmentation/organized_segmentation_tbb.cpp
)

set(dependencies
  Eigen3
)

set(library_name ${PROJECT_NAME})
add_library(${library_name} SHARED
  ${library_srcs}
  ${plugins_srcs}
  ${organized_segmentation_srcs}
)

target_link_libraries(${library_name}
  ${PCL_LIBRARIES}
  ${OPENNI_LIBRARIES}
  gtsam
  ${Boost_LIBRARIES}
  ${TBB_LIBRARIES}
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

target_include_directories(${library_name}
  PUBLIC
  ${GTSAM_INCLUDE_DIR}
  ${GTSAM_INCLUDE_DIR}
)

add_executable(omnimapper_test src/omnimapper_test.cpp)

ament_target_dependencies(omnimapper_test
  ${dependencies}
)

target_link_libraries(omnimapper_test
  ${library_name}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OPENNI_LIBRARIES}
  gtsam
  ${TBB_LIBRARIES}
)

install(TARGETS ${library_name}
  omnimapper_test
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)

ament_export_include_directories(include)
ament_export_libraries(
  ${library_name}
)
ament_package()