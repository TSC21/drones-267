Installing Optimised OpenCV On ARM
==================================

These instructions describe how to cross-compile a version of OpenCV optimised for performance on a multi-core ARMhf embedded system running a variant of Ubuntu 13.10.

Cross compilation is necessary because native compilation will take upwards of 2 hours, depending on the board in question.


Optimisations
-------------
The instructions below enable a number of optimisations:
+ libjpeg-turbo, an ARM optimised JPEG library to use in-place of libjpeg within OpenCV
+ TBB support, Thread Building Blocks (TBB) is an Intel maintained library that allows easy use of thread abstractions. [Certain OpenCV functions][opencv-libtbb] have support for this which will make them more performant on multi-core systems.
+ NEON/VFPv3 specific compilation flags to allow use of NEON intrinsics. Again, certain OpenCV functions may include NEON optimised code paths. gcc will also attempt to optimise generated code for NEON.

Instructions
------------
It's necessary to compile OpenCV from source, and optionally enable the ARM NEON extensions. You may also want to enable [TBB][tbb] support. You can cross compile using [these][2]instructions (I also followed a variant of the instructions in the script [here][3] to install). Building on a Linux Mint 16 host (= Ubuntu 13.10), I did the following:
```
cd ~/workspace
wget http://sourceforge.net/projects/opencvlibrary/files/latest/download
mv download opencv-2.4.8.zip
unzip opencv-2.4.8.zip
sudo apt-get install cmake gcc-arm-linux-gnueabihf g++-4.8-arm-linux-gnueabihf libtbb-dev
```

### Building libjpeg-turbo
Add `-fPIC` to CFLAGS, CPPFLAGS and CXXFLAGS.


Edit the cmake file and replace 4.6 with 4.8:
```
nano opencv-2.4.8/platforms/linux/arm-gnueabi.toolchain.cmake
```

Then run cmake, make and tar up the resulting files before scping to your target board:
```
mkdir build_hardfp
cd build_hardfp
cmake -DCMAKE_BUILD_TYPE=RELEASE -DWITH_V4L=ON -DBUILD_NEW_PYTHON_SUPPORT=OFF  -DENABLE_NEON=ON -DENABLE_VFPV3=ON -DWITH_TBB=YES -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_TOOLCHAIN_FILE=../arm-gnueabi.toolchain.cmake ../../..
make -j4
cd ~/workspace
tar -zcvf opencv-2.4.8.tgz opencv-2.4.8
scp opencv-2.4.8.tgz 192.168.1.10:~/workspace/
```

You'll need to install libtbb on the target computer too. A compiled armhf .deb is [here][libtbb] for Ubuntu distributions. Download both this and libtbb-dev and install using `dpkg -i <path/to/file>`.

Log into the target computer. Note that you should ensure the extracted files are in the same location as the computer they were compiled on (i.e. /home/user/workspace/opencv-2.4.8 on both compilation computer and target computer) - since cmake and make have many hardcoded paths.

```
ssh 192.168.1.10
cd ~/workspace
tar -xvf opencv-2.4.8.tgz
cd platforms/linux/build_hardfp
sudo make install
sudo sh -c 'echo "/usr/local/lib" > /etc/ld.so.conf.d/opencv.conf'
sudo ldconfig
```

[1]: http://docs.opencv.org/doc/tutorials/introduction/linux_install/linux_install.html
[2]: http://docs.opencv.org/doc/tutorials/introduction/crosscompilation/arm_crosscompile_with_cmake.html
[3]: https://help.ubuntu.com/community/OpenCV
[tbb]: https://www.threadingbuildingblocks.org/
[libtbb]: https://launchpad.net/ubuntu/saucy/armhf/libtbb2
[opencv-libtbb]: http://experienceopencv.blogspot.com/2011/07/parallelizing-loops-with-intel-thread.html
[libjpeg-turbo8]: http://packages.ubuntu.com/saucy/libjpeg-turbo8