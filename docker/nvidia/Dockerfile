#From inside this folder
#docker build -t cogrob/omnimapper-nvidia .

############################################################
# Dockerfile to build OmiMapper images
# Based on Ubuntu
############################################################

FROM cogrob/omnimapper-demo
MAINTAINER Cognitive Robotics "http://cogrob.org/"

RUN sudo apt-get update && sudo apt-get install -y \
	x-window-system \
	binutils \
	mesa-utils \
	module-init-tools

ADD nvidia-driver.run /tmp/nvidia-driver.run
RUN sudo sh /tmp/nvidia-driver.run -a -N --ui=none --no-kernel-module \
	&& sudo rm /tmp/nvidia-driver.run
