#!/bin/sh

# Get your current host nvidia driver version, e.g. 340.24
nvidia_version=$(cat /proc/driver/nvidia/version | head -n 1 | awk '{ print $8 }')

# We must use the same driver in the image as on the host
if test ! -f nvidia-driver.run; then
  nvidia_driver_uri=http://us.download.nvidia.com/XFree86/Linux-x86_64/${nvidia_version}/NVIDIA-Linux-x86_64-${nvidia_version}.run
  wget -O nvidia-driver.run $nvidia_driver_uri
fi

# docker build -t cogrob/omnimapper-nvidia:${nvidia_version} .
# docker tag cogrob/omnimapper-nvidia:${nvidia_version} cogrob/omnimapper-nvidia:latest
