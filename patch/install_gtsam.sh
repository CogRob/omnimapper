#!/usr/bin/env bash
set -e

git clone -b 3.2.2 https://bitbucket.org/gtborg/gtsam.git
cd gtsam
patch -p1 < ../gtsam_3_2_2_boost_1_65_patch.diff

mkdir build && cd build
cmake -DCMAKE_CXX_FLAGS="-fpermissive -w" ..
make -j`nproc`

make install
