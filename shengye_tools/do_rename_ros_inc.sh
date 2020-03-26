#!/bin/bash

# initialize a semaphore with a given number of tokens
open_sem(){
  mkfifo pipe-$$
  exec 3<>pipe-$$
  rm pipe-$$
  local i=$1
  for((;i>0;i--)); do
    printf %s 000 >&3
  done
}

# run the given command asynchronously and pop/push tokens
run_with_lock(){
  local x
  local EXIT_CODE
  # this read waits until there is something to read
  read -u 3 -n 3 x && ((0==x)) || exit $x
  (
   ( bash -c "$@" || EXIT_CODE=$?; )
  # push the return code of the command to the semaphore
  printf '%.3d' $EXIT_CODE >&3
  )&
}

SRC_FILES=( \
  omnimapper_ros/ar_marker_plugin.h \
  omnimapper_ros/canonical_scan.h \
  omnimapper_ros/canonical_scan_matcher_plugin.h \
  omnimapper_ros/csm_math_functions.h \
  omnimapper_ros/csm_visualizer.h \
  omnimapper_ros/error_evaluation_plugin.h \
  omnimapper_ros/get_transform_functor_tf.h \
  omnimapper_ros/omnimapper_ros.h \
  omnimapper_ros/omnimapper_ros_nodelet.h \
  omnimapper_ros/omnimapper_visualizer_rviz.h \
  omnimapper_ros/ros_tf_utils.h \
  omnimapper_ros/ros_time_utils.h \
  omnimapper_ros/tf_pose_plugin.h \
  omnimapper_ros/tum_data_error_plugin.h \
)

N=$(nproc)
INPUT_FILE=/home/shengye/CogRob/omnimapper/shengye_tools/omnimapper_clang_rename.yaml
SRC_PATH=/home/shengye/CogRob/omnimapper/ros/omnimapper_ros/include
OUTPUT_PATH=/home/shengye/CogRob/omnimapper/ros/omnimapper_ros/include_new
CLANG_ARGS="-DDISABLE_LIBUSB_1_0 -DDISABLE_PCAP -DDISABLE_PNG -DQT_CORE_LIB -DQT_GUI_LIB -DQT_NO_DEBUG -DQT_WIDGETS_LIB -DROSCONSOLE_BACKEND_LOG4CXX -DROS_BUILD_SHARED_LIBS=1 -DROS_PACKAGE_NAME=\\\"omnimapper_ros\\\" -Dqh_QHpointer -DvtkFiltersFlowPaths_AUTOINIT=\"1(vtkFiltersParallelFlowPaths)\" -DvtkIOExodus_AUTOINIT=\"1(vtkIOParallelExodus)\" -DvtkIOGeometry_AUTOINIT=\"1(vtkIOMPIParallel)\" -DvtkIOImage_AUTOINIT=\"1(vtkIOMPIImage)\" -DvtkIOParallel_AUTOINIT=\"1(vtkIOMPIParallel)\" -DvtkIOSQL_AUTOINIT=\"2(vtkIOMySQL,vtkIOPostgreSQL)\" -DvtkRenderingContext2D_AUTOINIT=\"1(vtkRenderingContextOpenGL)\" -DvtkRenderingCore_AUTOINIT=\"3(vtkInteractionStyle,vtkRenderingFreeType,vtkRenderingOpenGL)\" -DvtkRenderingFreeType_AUTOINIT=\"2(vtkRenderingFreeTypeFontConfig,vtkRenderingMatplotlib)\" -DvtkRenderingLIC_AUTOINIT=\"1(vtkRenderingParallelLIC)\" -DvtkRenderingVolume_AUTOINIT=\"1(vtkRenderingVolumeOpenGL)\" -I/home/shengye/catkin_ws/devel/.private/omnimapper_ros/include -I/usr/local/lib/cmake/GTSAM/../../../include -I/opt/intel/mkl/include -I/usr/local/lib/cmake/GTSAM/../../../include/gtsam/3rdparty/Eigen -isystem /usr/include/vtk-6.3 -isystem /usr/include/freetype2 -isystem /usr/lib/x86_64-linux-gnu/openmpi/include/openmpi -isystem /usr/lib/x86_64-linux-gnu/openmpi/include/openmpi/opal/mca/event/libevent2022/libevent -isystem /usr/lib/x86_64-linux-gnu/openmpi/include/openmpi/opal/mca/event/libevent2022/libevent/include -isystem /usr/lib/x86_64-linux-gnu/openmpi/include -isystem /usr/include/python2.7 -isystem /usr/include/x86_64-linux-gnu -isystem /usr/include/hdf5/openmpi -isystem /usr/include/libxml2 -isystem /usr/include/jsoncpp -isystem /usr/include/tcl -I/home/shengye/catkin_ws/src/omnimapper_ros/include -I/opt/ros/melodic/include -I/opt/ros/melodic/share/xmlrpcpp/cmake/../../../include/xmlrpcpp -isystem /usr/include/eigen3 -isystem /usr/local/include/pcl-1.9 -I/usr/local/include/gtsam/3rdparty/Eigen -isystem /usr/include/ni -isystem /usr/include/openni2 -isystem /usr/include/x86_64-linux-gnu/qt5 -isystem /usr/include/x86_64-linux-gnu/qt5/QtWidgets -isystem /usr/include/x86_64-linux-gnu/qt5/QtGui -isystem /usr/include/x86_64-linux-gnu/qt5/QtCore -isystem /usr/lib/x86_64-linux-gnu/qt5/mkspecs/linux-g++    -O3 -DNDEBUG -O3 -std=c++11 -fpermissive -w   -fPIC -std=gnu++11"

open_sem $N
for thing in "${SRC_FILES[@]}"; do
  run_with_lock "echo $thing; clang-rename-9 --force --extra-arg-before=-xc++ --input=$INPUT_FILE $SRC_PATH/$thing -- $CLANG_ARGS > $OUTPUT_PATH/$thing"
done
