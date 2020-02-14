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
find_package(Boost REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(PCL REQUIRED)

include_directories(
  include
  ${PCL_COMMON_INCLUDE_DIRS}
)

set (library_srcs
  src/omnimapper_base.cpp
  src/time.cpp
  src/transform_tools.cpp
)

set (plugins_srcs
  src/plugins/icp_plugin.cpp
  src/plugins/no_motion_pose_plugin.cpp
)

set (organized_segmentation_srcs
  src/organized_segmentation/organized_segmentation_tbb.cpp
)

add_library(${PROJECT_NAME} SHARED
  ${library_srcs}
  ${plugins_srcs}
  ${organized_segmentation_srcs}
)

target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
  ${GTSAM_LIBRARIES}
  ${TBB_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}
  Eigen3
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  ${GTSAM_INCLUDE_DIR}
  ${GTSAM_INCLUDE_DIR}
)

install(TARGETS ${PROJECT_NAME}
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

ament_package()