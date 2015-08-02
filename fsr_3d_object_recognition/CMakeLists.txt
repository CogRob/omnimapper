cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(fsr_3d_object_recognition)

set(CMAKE_BUILD_TYPE Debug)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(include)
set(incs_fsr include/fsr_threedorlib/fsr_types.h include/fsr_threedorlib/fsr_conflictgraph.h include/fsr_threedorlib/fsr_hashdescription.h include/fsr_threedorlib/fsr_recognition.h)
set(srcs_fsr src/fsr_types.cpp src/fsr_hashdescription.cpp src/fsr_recognition.cpp)

#set(incs_seg include/organized_segmentation_tools/organized_segmentation_tbb.h)
#set(srcs_seg src/organized_segmentation_tbb.cpp)

add_library(fsr_threedorlib SHARED ${incs_fsr} ${srcs_fsr})
#add_library(segtools SHARED ${incs_seg} ${srcs_seg})
#target_link_libraries(fsr_threedorlib segtools ${PCL_LIBRARIES} tbb tbbmalloc)
target_link_libraries(fsr_threedorlib ${PCL_LIBRARIES} tbb tbbmalloc)

#add_executable(fsr_or src/or_main.cpp)
#target_link_libraries(fsr_or fsr_threedorlib)

add_executable(fsr_hash_creation_app src/fsr_hash_creation_app.cpp)
target_link_libraries(fsr_hash_creation_app fsr_threedorlib)

add_executable(disp_pcdtxt src/dpcd_main.cpp)
target_link_libraries(disp_pcdtxt fsr_threedorlib)

#add_executable(saveseg src/saveseg_main.cpp)
#target_link_libraries(saveseg segtools)
