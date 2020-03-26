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
  # this read waits until there is something to read
  read -u 3 -n 3 x && ((0==x)) || exit $x
  (
   ( bash -c "$@"; )
  # push the return code of the command to the semaphore
  printf '%.3d' $? >&3
  )&
}

N=12
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

open_sem $N
INPUT_FILE=/home/shengye/CogRob/omnimapper/shengye_tools/omnimapper_clang_rename.yaml
BUILD_PATH=/home/shengye/CogRob/omnimapper/build
SRC_PATH=/home/shengye/CogRob/omnimapper/src
OUTPUT_PATH=/home/shengye/CogRob/omnimapper/src_new
for thing in "${SRC_FILES[@]}"; do
  run_with_lock "clang-rename-9 --force --input=$INPUT_FILE -p=$BUILD_PATH $SRC_PATH/$thing > $OUTPUT_PATH/$THING"
done
