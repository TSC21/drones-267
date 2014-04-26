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
It's necessary to compile OpenCV from source and enable the ARM NEON extensions. You may also want to enable [TBB][tbb] support. You can cross compile using [these][2]instructions (I also followed a variant of the instructions in the script [here][3] to install). Building on a Linux Mint 16 host (= Ubuntu 13.10), I did the following.

### Getting dependencies
```
sudo apt-get install cmake gcc-arm-linux-gnueabihf g++-4.8-arm-linux-gnueabihf libtbb-dev
```

### Building libjpeg-turbo
The procedure here is to cross-compile libjpeg-turbo from source and to statically link the libraries into OpenCV. 

First, grab the latest source. I use the `libjpeg-turbo_1.3.0.orig.tar.gz` tarball from the [Ubuntu package page][libjpeg-turbo8] because of issues getting the autoreconf command to work with the latest source from the official site.

```
cd ~/workspace
wget http://archive.ubuntu.com/ubuntu/pool/main/libj/libjpeg-turbo/libjpeg-turbo_1.3.0.orig.tar.gz
tar -xvf libjpeg-turbo_1.3.0.orig.tar.gz
cd libjpeg-turbo-1.3.0
```

It's then necessary to run configure with a number of flags specifying the cross-compiler, target architecture and where you'd like to the output build to go. Change the --prefix path below as appropriate.
```
mkdir build
./configure --host=arm-linux-gnueabihf CC=arm-linux-gnueabihf-gcc AR=arm-linux-gnueabihf-ar \
          STRIP=arm-linux-gnueabihf-strip RANLIB=arm-linux-gnueabihf-ranlib \
          --prefix=/home/user/workspace/libjpeg-turbo-1.3.0/build/  --enable-static
```

It's then necessary to add the `-fPIC` flag to various FLAGS variables in the generated Makefile. You should be able to pass these as an option to configure but my attempts to do so using `CPPFLAGS='-fPIC'` didn't work.

Go into the Makefile with `vim Makefile` and add `-fPIC` to the end of the CFLAGS, CPPFLAGS and CXXFLAGS lines. (These are probably not all needed but I was tired of repeatedly compiling these libraries :).)

You will need to install this on your target machine. Something like the following (assuming you've copied the files over already):
```
mkdir /opt/libjpeg-turbo/
sudo cp -rf ~/workspace/libjpeg-turbo-1.3.0/build/* /opt/libjpeg-turbo/
sudo sh -c 'echo "/opt/libjpeg-turbo/lib" >> /etc/ld.so.conf.d/libjpeg-turbo.conf'
sudo ldconfig
```

### Building OpenCV

First, download the latest version.
```
cd ~/workspace
wget http://sourceforge.net/projects/opencvlibrary/files/latest/download
mv download opencv-2.4.8.zip
unzip opencv-2.4.8.zip
```

Then, edit the cmake file and change the `GCC_COMPILER_VERSION` at the top from `4.6` to `4.8`. I've seen considerably better performance with the 4.8 version of the cross-compiler:
```
nano opencv-2.4.8/platforms/linux/arm-gnueabi.toolchain.cmake
```

Then run cmake, make and tar up the resulting files before scping to your target board:
```
cd opencv-2.4.8/platforms/linux
mkdir build_hardfp
cd build_hardfp
cmake -DCMAKE_BUILD_TYPE=RELEASE -DWITH_V4L=ON -DBUILD_NEW_PYTHON_SUPPORT=OFF  -DENABLE_NEON=ON -DENABLE_VFPV3=ON -DWITH_TBB=ON -DBUILD_TBB=ON -DWITH_JPEG=ON -DBUILD_JPEG=OFF -DJPEG_INCLUDE_DIR=/home/user/workspace/libjpeg-turbo-1.3.0/build/include/ -DJPEG_LIBRARY=/home/user/workspace/libjpeg-turbo-1.3.0/build/lib/libjpeg.a -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_TOOLCHAIN_FILE=../arm-gnueabi.toolchain.cmake ../../..
```
You can verify that it picked up the correct libjpeg-turbo library and is compiling with the right modules at this stage.

```
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