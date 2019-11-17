# Patches for Omnimapper

## GTSAM

We are using GTSAM 3.2.2 with a patch for boost that comes with Ubuntu 18.04.
To install GTSAM:

```
git clone -b 3.2.2 https://bitbucket.org/gtborg/gtsam.git
cd gtsam
patch -p1 < <PATH_TO_PATCH>/gtsam_3_2_2_boost_1_65_patch.diff

mkdir build && cd build
cmake -DCMAKE_CXX_FLAGS="-fpermissive -w" ..
make -j`nproc`
sudo make install
```

NOTE: Newer versions of GTSAM may be available [here](https://github.com/borglab/gtsam).
They might no longer require a patch. GTSAM 3.2.x is known more likely to work
properly with Omnimapper than GTSAM 4.0.x as of now.
