#From inside this folder
# docker build -t cogrob/omnimapper-dep .
# docker run -t -i -v /path/to/local/omnimapper/repo:~/git/omnimapper  cogrob/omnimapper-dep /bin/bash
# docker export omnimapper-dep | gzip -c > omnimapper-dep.tgz
# docker import omnimapper-dep < omnimapper-dep.tgz

############################################################
# Dockerfile to build OmiMapper images
# Based on Ubuntu
############################################################

FROM cogrob/omnimapper-dep
MAINTAINER Cognitive Robotics "http://cogrob.org/"

RUN apt-get install -y \
	cmake \
	cmake-curses-gui

RUN apt-get install -y \
	libboost-all-dev \
	libflann-dev \
	libgsl0-dev \
	libgoogle-perftools-dev

RUN apt-get install -y \
	libeigen3-dev \
	ros-indigo-perception-pcl \
	ros-indigo-openni-launch \
	ros-indigo-turtlebot-simulator