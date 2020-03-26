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
  BoundedPlane3.cpp \
  BoundedPlaneFactor.cpp \
  bounded_plane_plugin.cpp \
  icp_pose_plugin.cpp \
  isam_splice_test.cpp \
  no_motion_pose_plugin.cpp \
  object.cpp \
  object_discovery.cpp \
  object_plugin.cpp \
  object_recognition.cpp \
  object_segment_propagation.cpp \
  omnimapper_base.cpp \
  omnimapper_handheld_demo.cpp \
  omnimapper_handheld_pcd_demo.cpp \
  omnimapper_plane_test.cpp \
  omnimapper_simple_icp_demo.cpp \
  omnimapper_test.cpp \
  omnimapper_visualizer_pcl.cpp \
  organized_feature_extraction.cpp \
  organized_feature_extraction_demo.cpp \
  organized_feature_extraction_demo_tbb.cpp \
  organized_feature_extraction_tbb.cpp \
  OrientedPlane3.cpp \
  OrientedPlane3Factor.cpp \
  plane.cpp \
  plane_factor.cpp \
  plane_plugin.cpp \
  tbb_test.cpp \
  time.cpp \
  transform_helpers.cpp \
  tsdf_output_plugin.cpp \
)

N=$(nproc)
INPUT_FILE=/home/shengye/CogRob/omnimapper/shengye_tools/omnimapper_clang_rename.yaml
SRC_PATH=/home/shengye/CogRob/omnimapper/src
OUTPUT_PATH=/home/shengye/CogRob/omnimapper/src_new
CLANG_ARGS="-Dqh_QHpointer -DvtkFiltersFlowPaths_AUTOINIT=\"1(vtkFiltersParallelFlowPaths)\" -DvtkIOExodus_AUTOINIT=\"1(vtkIOParallelExodus)\" -DvtkIOGeometry_AUTOINIT=\"1(vtkIOMPIParallel)\" -DvtkIOImage_AUTOINIT=\"1(vtkIOMPIImage)\" -DvtkIOParallel_AUTOINIT=\"1(vtkIOMPIParallel)\" -DvtkIOSQL_AUTOINIT=\"2(vtkIOMySQL,vtkIOPostgreSQL)\" -DvtkRenderingContext2D_AUTOINIT=\"1(vtkRenderingContextOpenGL)\" -DvtkRenderingCore_AUTOINIT=\"3(vtkInteractionStyle,vtkRenderingFreeType,vtkRenderingOpenGL)\" -DvtkRenderingFreeType_AUTOINIT=\"2(vtkRenderingFreeTypeFontConfig,vtkRenderingMatplotlib)\" -DvtkRenderingLIC_AUTOINIT=\"1(vtkRenderingParallelLIC)\" -DvtkRenderingVolume_AUTOINIT=\"1(vtkRenderingVolumeOpenGL)\" -I/usr/include/vtk-6.3 -I/usr/include/freetype2 -I/usr/lib/x86_64-linux-gnu/openmpi/include/openmpi -I/usr/lib/x86_64-linux-gnu/openmpi/include/openmpi/opal/mca/event/libevent2022/libevent -I/usr/lib/x86_64-linux-gnu/openmpi/include/openmpi/opal/mca/event/libevent2022/libevent/include -I/usr/lib/x86_64-linux-gnu/openmpi/include -I/usr/include/python2.7 -I/usr/include/x86_64-linux-gnu -I/usr/include/hdf5/openmpi -I/usr/include/libxml2 -I/usr/include/jsoncpp -I/usr/include/tcl -I/usr/local/include/pcl-1.9 -I/usr/local/include/gtsam/3rdparty/Eigen -I/usr/include/ni -I/usr/include/openni2 -I/opt/intel/mkl/include -I/usr/local/include -I/home/shengye/CogRob/omnimapper/include    -O3 -DNDEBUG -fpermissive -w -O3 -std=c++11"

open_sem $N
for thing in "${SRC_FILES[@]}"; do
  run_with_lock "echo $thing; clang-rename-9 --force --extra-arg-before=-xc++ --input=$INPUT_FILE $SRC_PATH/$thing -- $CLANG_ARGS > $OUTPUT_PATH/$thing"
done
