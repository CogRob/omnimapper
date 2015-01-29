# docker build -t cogrob/omnimapper-dep .
# docker export omnimapper-dep | gzip -c > omnimapper-dep.tgz
# docker import omnimapper-dep < omnimapper-dep.tgz

############################################################
# Dockerfile to build OmiMapper images
# Based on Ubuntu
############################################################

FROM ubuntu:14.04
MAINTAINER Cognitive Robotics "http://cogrob.org/"

# Intall ROS
RUN apt-get update && apt-get install -y \
	software-properties-common \
	wget

RUN wget http://packages.ros.org/ros.key -O - | apt-key add -
RUN echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list
RUN apt-get update && apt-get install -y \
	ros-indigo-desktop-full

RUN rosdep init \
	&& rosdep update \
	&& echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc

# Intall some basic CLI tools
RUN apt-get install -y \
	curl \
	screen \
	byobu \
	fish \
	git \
	nano \
	glances

CMD ["bash"]