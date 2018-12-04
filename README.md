OmniMapper
==========

OmniMapper is a modular, multimodal framework for solving SLAM problems. It provides a plugin-based architecture for easy integration of new sensors and feature types.

For more information on the design of OmniMapper, please refer to the publication: [OmniMapper: A Modular Multimodal Mapping Framework](https://ieeexplore.ieee.org/document/6907122).


# Installation

This version of OmniMapper has been tested on Ubuntu 16.04 with ROS Kinetic.

## Dependencies

Currently OmniMapper primarily depends on GTSAM 4.0 and PCL. These libraries also have a variety of dependencies (many of which you may already have installed).

Install Cmake, Boost, Git (used by all libraries):
```
sudo apt-get install cmake libboost-all-dev git
```

Install PCL dependencies (FLANN and VTK):
```
sudo apt-get install libflann-dev libvtk6-dev
```

Install Intel Thread Building Blocks (TBB), used by GTSAM and OmniMapper:
```
sudo apt-get install libtbb-dev
```

Install Google Perf Tools  (used by OmniMapper):
```
sudo apt-get install libgoogle-perftools-dev libgsl0-dev
```

Install GSL (used by OmniMapper ROS):
```
sudo apt-get install libgsl0-dev
```


### Install GTSAM 4.0

Clone the GTSAM repository from BitBucket and checkout the 4.0.0 tag:

```
git clone https://antowilby@bitbucket.org/gtborg/gtsam.git
cd gtsam
git checkout tags/4.0.0-alpha2 -b 4.0.0
```

Compile GTSAM and install:

```
mkdir build && cd build
cmake ..
make -j`nproc`
sudo make install
```

### Install PCL

Install the most recent version of PCL from Github:

```
git clone https://github.com/PointCloudLibrary/pcl.git
mkdir build && cd build
```

We need to build PCL against the Eigen library included in GTSAM as it includes custom patches. Modify the path below to your cloned GTSAM repo:
```
cmake -DEIGEN_INCLUDE_DIR="/path/to/gtsam-4.0/gtsam/3rdparty/Eigen" ..
make -j`nproc`
sudo make install
```


### Install OmniMapper

Get the most recent version of OmniMapper from Github:

```
git clone https://github.com/CogRob/omnimapper.git
```

From the omnimapper directory, compile OmniMapper and install:

```
mkdir build && cd build
```

As with PCL, we need to build against the Eigen library included in GTSAM as it includes custom patches. Modify the path below to your cloned GTSAM repo:
```
cmake -DEIGEN_INCLUDE_DIR="/path/to/gtsam-4.0/gtsam/3rdparty/Eigen" ..
make -j`nproc`
sudo make install
```

### Install ROS

If you don't already have ROS installed on your system, follow the instructions for installing ROS Kinetic [here](http://wiki.ros.org/kinetic/Installation).

Also install the following packages for PCL and OpenNI-compliant RGB-D sensors:
```
sudo apt-get install ros-kinetic-perception-pcl ros-kinetic-openni-launch
```

### Install OmniMapper ROS Package

In your ROS workspace, clone the omnimapper_ros package:

```
https://github.com/CogRob/omnimapper_ros.git
```

From the root of your workspace, run:
```
catkin_make
```

You should now be ready to run OmniMapper. See the next section for instructions.


# Running Omnimapper



Further documentation available at:
https://github.com/CogRob/omnimapper/wiki
