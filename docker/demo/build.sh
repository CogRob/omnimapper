echo "if [ -f ~/.bash_aliases ]; then
    . ~/.bash_aliases
fi" >> ~/.bashrc
source /opt/ros/indigo/setup.bash
echo "export ROS_PACKAGE_PATH=/home/dox/omnimapper/ros/:${ROS_PACKAGE_PATH}" >> ~/.bash_aliases
source ~/.bashrc

# source /opt/ros/indigo/setup.bash
# export ROS_PACKAGE_PATH=/home/dox/omnimapper/ros/:${ROS_PACKAGE_PATH}

sudo rosdep init

rosdep update

roscd omnimapper_ros

rosmake

# mkdir build 

cd build 

cmake -DEIGEN_INCLUDE_DIRS="/home/dox/gtsam/gtsam/3rdparty/Eigen" .. 

make -j8