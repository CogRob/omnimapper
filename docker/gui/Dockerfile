#From inside this folder
# docker build -t cogrob/omnimapper-gui .

############################################################
# Dockerfile to build OmiMapper images
# Based on Ubuntu
############################################################

FROM cogrob/omnimapper-dev
MAINTAINER Cognitive Robotics "http://cogrob.org/"

# Intall some basic GUI and sound libs
RUN apt-get install -y \
		xz-utils file locales dbus-x11 pulseaudio dmz-cursor-theme \
		fonts-dejavu fonts-liberation hicolor-icon-theme \
		libcanberra-gtk3-0 libcanberra-gtk-module libcanberra-gtk3-module \
		libasound2 libgtk2.0-0 libdbus-glib-1-2 libxt6 libexif12 \
		libgl1-mesa-glx libgl1-mesa-dri \
 	&& update-locale LANG=C.UTF-8 LC_MESSAGES=POSIX

# Intall some basic GUI tools
RUN apt-get install -y \
	terminator \
	cmake-qt-gui \
	gedit