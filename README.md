# Omnimapper

## clang-format
```
clang-format -i -style=google **/*.cpp **/*.h **/*.hpp
```

## clang-rename
```
export COGROB_OMNIMAPPER_REPO=/home/shengye/CogRob/omnimapper
clang-rename-9 -i --force -extra-arg-before=-xc++ \
  --input=$COGROB_OMNIMAPPER_REPO/shengye_tools/omnimapper_base_clang_rename.yaml \
  $COGROB_OMNIMAPPER_REPO/**/*.h \
  $COGROB_OMNIMAPPER_REPO/**/*.cpp \
  -- \
  -I$COGROB_OMNIMAPPER_REPO/include \
  -I$COGROB_OMNIMAPPER_REPO/ros/omnimapper_ros/include \
  -I/opt/intel/compilers_and_libraries_2019.5.281/linux/mkl/include \
  -I/usr/local/include/pcl-1.9 \
  -I/usr/local/include/gtsam/3rdparty/Eigen \
  -I/usr/include/ni \
  -I/opt/ros/melodic/include \
  -I/usr/include/vtk-6.3
```

