#From inside this folder
# docker build -t cogrob/omnimapper-dox .

############################################################
# Dockerfile to build OmiMapper images
# Based on Ubuntu
############################################################

FROM ruffsl/omnimapper-gui
MAINTAINER Cognitive Robotics "http://cogrob.org/"

#Install qtcreator IDE
# RUN apt-get install -y qtcreator

#Install firefox browser for network debuging
RUN apt-get install -y firefox 

#Install Google Chrome, run using 'google-chrome --sand-box'
# RUN wget -q -O - "https://dl-ssl.google.com/linux/linux_signing_key.pub" | sudo apt-key add - \
#  && echo "deb http://dl.google.com/linux/chrome/deb/ stable main" > /etc/apt/sources.list.d/google-chrome.list \
#  && apt-get update \
#  && apt-get install -y google-chrome-stable

#Add basic user
ENV USERNAME dox
ENV HOME /home/$USERNAME
ENV PULSE_SERVER /run/pulse/native
RUN mkdir -p $HOME \
	&& useradd $USERNAME \
	&& echo "$USERNAME:$USERNAME" | chpasswd \
	&& echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
	&& chown -R $USERNAME:$USERNAME $HOME

#Setup .bashrc for ROS
RUN echo "source /opt/ros/indigo/setup.bash" >> /home/$USERNAME/.bashrc 

#Fix for qt and X server errors
RUN echo "export QT_X11_NO_MITSHM=1" >> /home/$USERNAME/.bashrc 

#Change user
USER dox