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

Install Intel Thread Building Blocks (TBB) and GSL (used by GTSAM and OmniMapper):
```
sudo apt-get install libtbb-dev libgsl-dev
```

Install Google Perf Tools  (used by OmniMapper):
```
sudo apt-get install libgoogle-perftools-dev
```


### Install GTSAM 4.0

Clone the GTSAM repository from BitBucket and checkout the most recently tested commit:

```
git clone https://antowilby@bitbucket.org/gtborg/gtsam.git
cd gtsam
git checkout 6f8bfe0
```

Compile GTSAM and install:

```
mkdir build && cd build
cmake ..
make -j`nproc`
sudo make install
```

### Install PCL

Install PCL 1.7.2 from Github:

```
git clone https://github.com/PointCloudLibrary/pcl.git
git checkout tags/pcl-1.7.2 -b 1.7.2
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
cmake -DEIGEN_INCLUDE_DIRS="/path/to/gtsam-4.0/gtsam/3rdparty/Eigen" ..
make -j`nproc`
sudo make install
```

### Install ROS and OmniMapper ROS

If you don't already have ROS installed on your system, follow the instructions for installing ROS Kinetic [here](http://wiki.ros.org/kinetic/Installation).

Also install the following packages for PCL and OpenNI-compliant RGB-D sensors:
```
sudo apt-get install ros-kinetic-perception-pcl ros-kinetic-openni-launch
```

Now, clone the omnimapper_ros repository into your ROS workspace:
```
https://github.com/CogRob/omnimapper_ros.git
```


#### Get CSM

Clone the most recent version of CSM (Canonical Scan Matcher) from Github into your ROS workspace:
```
git clone https://github.com/AndreaCensi/csm.git
```

We must tell omnimapper_ros where to find CSM. We have provided some config files in the omnimapper_ros repository to make this easier.

From omnimapper_ros/csm_config, copy the `cmake` folder into the root folder of the csm package.

Next, copy the local-csm-config.cmake file from omnimapper_ros/csm_config into csm/local_config. Rename this file to local-your-computer's-hostname.cmake (find your hostname by running `hostname` at the command prompt).

Now, rom the CSM directory, run the `install_quickstart.sh` script.

If this is successful, CSM has now been compiled and installed to `csm/deploy`.


#### Install OmniMapper ROS Package

Now, edit the CMakeLists.txt for omnimapper_ros. Find the line where CSM_DIR is set and change the path to where your CSM is installed, e.g. /path/to/csm/deploy/lib/csm. This tells catkin where to find the cmake config file for CSM, and should allow omnimapper_ros to compile with no issues.

Also, copy the `libcsm.so` file from `csm/deploy/lib`,  to `/usr/local/lib`.


(Yes, this is annoying and hacky, and hopefully will be updated to a better installation process sometime in the future.)

From the root of your ROS workspace, run:
```
catkin_make
```

You should now be ready to run OmniMapper. See the next section for instructions.


# Running Omnimapper



Further documentation available at:
https://github.com/CogRob/omnimapper/wiki
