#From inside this folder
# docker build -t cogrob/omnimapper-demo .

############################################################
# Dockerfile to build OmiMapper images
# Based on Ubuntu
############################################################

FROM cogrob/omnimapper-dox
MAINTAINER Cognitive Robotics "http://cogrob.org/"

USER dox

# download and unzip gtsam
ENV GTSAM_VERSION 3.2.0
WORKDIR /home/dox
RUN wget https://research.cc.gatech.edu/borg/sites/edu.borg/files/downloads/gtsam-${GTSAM_VERSION}.tgz \
	&& mkdir gtsam \
	&& tar -xvzf gtsam-${GTSAM_VERSION}.tgz --strip=1 -C gtsam/ \
	&& rm gtsam-${GTSAM_VERSION}.tgz

# build and install gtsam
RUN cd /home/dox/gtsam/ \
	&& mkdir build \
	&& cd build \
	&& cmake .. \
	&& make -j8 \
	&& sudo make install

COPY HouseholderQR.h /usr/include/eigen3/Eigen/src/QR/HouseholderQR.h
COPY CommaInitializer.h /usr/include/eigen3/Eigen/src/CommaInitializer.h
RUN sudo chmod 644 /usr/include/eigen3/Eigen/src/QR/HouseholderQR.h
RUN sudo chmod 644 /usr/include/eigen3/Eigen/src/CommaInitializer.h

# clone omnimapper project
WORKDIR /home/dox
RUN git clone https://github.com/CognitiveRobotics/omnimapper.git

# build omnimapper project
RUN cd /home/dox/omnimapper/ \
	&& mkdir build \
	&& cd build \
	&& cmake .. \
	&& make -j8

# build omnimapper ros wrapper
# Need to use script to use easly use local variables set from .bashrc
WORKDIR /home/dox
RUN sudo chown dox /home/dox/.bashrc

ADD build.sh /home/dox/build.sh
RUN sudo chown dox /home/dox/build.sh \
	&& bash /home/dox/build.sh