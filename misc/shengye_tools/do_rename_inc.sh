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
  omnimapper/pose_plugin.h \
  omnimapper/tsdf_output_plugin.h \
  omnimapper/impl/geometry.hpp \
  omnimapper/plane.h \
  omnimapper/object_plugin.h \
  omnimapper/icp_pose_plugin.h \
  omnimapper/pose_chain.h \
  omnimapper/organized_feature_extraction_tbb.h \
  omnimapper/omnimapper_visualizer_pcl.h \
  omnimapper/bounded_plane_plugin.h \
  omnimapper/BoundedPlane3.h \
  omnimapper/OrientedPlane3.h \
  omnimapper/get_transform_functor.h \
  omnimapper/BoundedPlaneFactor.h \
  omnimapper/output_plugin.h \
  omnimapper/object_segment_propagation.h \
  omnimapper/measurement_plugin.h \
  omnimapper/time.h \
  omnimapper/object_recognition.h \
  omnimapper/organized_feature_extraction.h \
  omnimapper/object.h \
  omnimapper/plane_plugin.h \
  omnimapper/no_motion_pose_plugin.h \
  omnimapper/plane_factor.h \
  omnimapper/landmark_factor.h \
  omnimapper/transform_helpers.h \
  omnimapper/object_discovery.h \
  omnimapper/geometry.h \
  omnimapper/omnimapper_base.h \
  omnimapper/3rdparty/eigen_extensions.h \
  omnimapper/3rdparty/distortion_model_standalone.h \
  omnimapper/OrientedPlane3Factor.h \
)

N=$(nproc)
INPUT_FILE=/home/shengye/CogRob/omnimapper/misc/shengye_tools/omnimapper_clang_rename.yaml
SRC_PATH=/home/shengye/CogRob/omnimapper/include
OUTPUT_PATH=/home/shengye/CogRob/omnimapper/include_new
CLANG_ARGS="-Dqh_QHpointer -DvtkFiltersFlowPaths_AUTOINIT=\"1(vtkFiltersParallelFlowPaths)\" -DvtkIOExodus_AUTOINIT=\"1(vtkIOParallelExodus)\" -DvtkIOGeometry_AUTOINIT=\"1(vtkIOMPIParallel)\" -DvtkIOImage_AUTOINIT=\"1(vtkIOMPIImage)\" -DvtkIOParallel_AUTOINIT=\"1(vtkIOMPIParallel)\" -DvtkIOSQL_AUTOINIT=\"2(vtkIOMySQL,vtkIOPostgreSQL)\" -DvtkRenderingContext2D_AUTOINIT=\"1(vtkRenderingContextOpenGL)\" -DvtkRenderingCore_AUTOINIT=\"3(vtkInteractionStyle,vtkRenderingFreeType,vtkRenderingOpenGL)\" -DvtkRenderingFreeType_AUTOINIT=\"2(vtkRenderingFreeTypeFontConfig,vtkRenderingMatplotlib)\" -DvtkRenderingLIC_AUTOINIT=\"1(vtkRenderingParallelLIC)\" -DvtkRenderingVolume_AUTOINIT=\"1(vtkRenderingVolumeOpenGL)\" -I/usr/include/vtk-6.3 -I/usr/include/freetype2 -I/usr/lib/x86_64-linux-gnu/openmpi/include/openmpi -I/usr/lib/x86_64-linux-gnu/openmpi/include/openmpi/opal/mca/event/libevent2022/libevent -I/usr/lib/x86_64-linux-gnu/openmpi/include/openmpi/opal/mca/event/libevent2022/libevent/include -I/usr/lib/x86_64-linux-gnu/openmpi/include -I/usr/include/python2.7 -I/usr/include/x86_64-linux-gnu -I/usr/include/hdf5/openmpi -I/usr/include/libxml2 -I/usr/include/jsoncpp -I/usr/include/tcl -I/usr/local/include/pcl-1.9 -I/usr/local/include/gtsam/3rdparty/Eigen -I/usr/include/ni -I/usr/include/openni2 -I/opt/intel/mkl/include -I/usr/local/include -I/home/shengye/CogRob/omnimapper/include    -O3 -DNDEBUG -fpermissive -w -O3 -std=c++11"

open_sem $N
for thing in "${SRC_FILES[@]}"; do
  run_with_lock "echo $thing; clang-rename-9 --force --extra-arg-before=-xc++ --input=$INPUT_FILE $SRC_PATH/$thing -- $CLANG_ARGS > $OUTPUT_PATH/$thing"
done
