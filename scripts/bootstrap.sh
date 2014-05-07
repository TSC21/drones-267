#!/usr/bin/env bash

echo "INSTALLING ROS"
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu raring main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
apt-get update
apt-get -y install ros-hydro-ros-base
rosdep init
rosdep update
sudo -u vagrant echo "source /opt/ros/hydro/setup.bash" >> /home/vagrant/.bashrc
source /opt/ros/hydro/setup.bash

ln -s /usr/lib/x86_64-linux-gnu/libboost_signals.so /usr/lib/libboost_signals-mt.so
ln -s /usr/lib/x86_64-linux-gnu/libboost_filesystem.so /usr/lib/libboost_filesystem-mt.so
ln -s /usr/lib/x86_64-linux-gnu/libboost_regex.so /usr/lib/libboost_regex-mt.so
ln -s /usr/lib/x86_64-linux-gnu/libboost_date_time.so /usr/lib/libboost_date_time-mt.so
ln -s /usr/lib/x86_64-linux-gnu/libboost_system.so /usr/lib/libboost_system-mt.so
ln -s /usr/lib/x86_64-linux-gnu/libboost_thread.so /usr/lib/libboost_thread-mt.so

echo "SETTING UP CATKIN WORKSPACE"
mkdir -p /home/vagrant/catkin_ws/src
cd /home/vagrant/catkin_ws/src
catkin_init_workspace

cd ../
catkin_make
sudo -u vagrant echo "source /home/vagrant/catkin_ws/devel/setup.bash" >> /home/vagrant/.bashrc

ln -s /vagrant /home/vagrant/catkin_ws/src/drones-267
chown -R vagrant.vagrant /home/vagrant/catkin_ws

cd ~§
echo "INSTALLING OPEN CV"
version="$(wget -q -O - http://sourceforge.net/projects/opencvlibrary/files/opencv-unix | egrep -m1 -o '\"[0-9](\.[0-9])+' | cut -c2-)"
echo "Installing OpenCV" $version
mkdir OpenCV
cd OpenCV
echo "Removing any pre-installed ffmpeg and x264"
sudo apt-get -yqq remove ffmpeg x264 libx264-dev
echo "Installing Dependencies"
apt-get -yqq install libopencv-dev 
apt-get -yqq install build-essential checkinstall cmake pkg-config yasm libopenexr6 libtiff4-dev libjpeg-dev libjasper-dev libavcodec-dev libavformat-dev libswscale-dev libdc1394-22-dev libxine-dev libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev libv4l-dev python-dev python-numpy libtbb-dev libqt4-dev libgtk2.0-dev libfaac-dev libmp3lame-dev libopencore-amrnb-dev libopencore-amrwb-dev libtheora-dev libvorbis-dev libxvidcore-dev x264 v4l-utils ffmpeg unzip
echo "Downloading OpenCV" $version
wget -O OpenCV-$version.zip http://sourceforge.net/projects/opencvlibrary/files/opencv-unix/$version/opencv-"$version".zip/download
echo "Installing OpenCV" $version
unzip OpenCV-$version.zip
cd opencv-$version
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D WITH_TBB=ON -D BUILD_NEW_PYTHON_SUPPORT=ON -D WITH_V4L=ON -D INSTALL_C_EXAMPLES=ON -D INSTALL_PYTHON_EXAMPLES=ON -D BUILD_EXAMPLES=ON -D WITH_QT=ON -D WITH_OPENGL=ON ..
make -j2
make install
sh -c 'echo "/usr/local/lib" > /etc/ld.so.conf.d/opencv.conf'
ldconfig
echo "OpenCV" $version "ready to be used"

echo "INSTALLING GIT"
apt-get -yqq install git

