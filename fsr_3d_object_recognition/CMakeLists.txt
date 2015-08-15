cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(fsr_3d_object_recognition)

#set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Release)

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

add_definitions(-DNDEBUG)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(include)
set(incs_fsr
include/fsr_threedorlib/fsr_types.h
include/fsr_threedorlib/fsr_conflictgraph.h
include/fsr_threedorlib/fsr_recognition_helper.h
include/fsr_threedorlib/fsr_hashdescription.h
include/fsr_threedorlib/fsr_recognition.h)

set(srcs_fsr
src/fsr_types.cpp
src/fsr_recognition_helper.cpp
src/fsr_hashdescription.cpp
src/fsr_recognition.cpp)

add_library(fsr_threedorlib SHARED ${incs_fsr} ${srcs_fsr})
target_link_libraries(fsr_threedorlib ${PCL_LIBRARIES} tbb tbbmalloc)



add_executable(app_fsr_hash_creation src/fsr_hash_creation_app.cpp)
target_link_libraries(app_fsr_hash_creation fsr_threedorlib)

add_executable(app_fsr_recognition src/fsr_recognition_app.cpp)
target_link_libraries(app_fsr_recognition fsr_threedorlib)
