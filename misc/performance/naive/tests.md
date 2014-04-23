Description of Tests Run
========================

We ran performance tests on our naive serial implementation by profiling the amount of time taking at various points of our pose estimation routine in chunks of 20 frames. This was performed for versions of OpenCV 2.4.8 compiled with and without ARM specific optimizations. See [here][1] for cross compilation instructions.


Testing Environment
-------------------
We used a large fixed size landing pad for all tests of 80 cm by 80 cm, as described in the "A Vision System for Landing an Unmanned Aerial Vehicle" on which this project is based.

All tests were run indoors in a room without windows and constant fluorescent tube light.

No other user space processes were running on each computer aside from:
- roscore
- rosrun pose_estimator PoseEstimator

For each test we take 10 samples of 20 frame timing chunks, discarding the first 20 frame chunk (since the camera changes exposure and focus settings) and average the results.

Beaglebone Black
----------------

### Specification
+ AM335x 1GHz ARM® Cortex-A8
+ 512MB DDR3 RAM
+ 2GB 8-bit eMMC on-board flash storage
+ 3D graphics accelerator
+ NEON floating-point accelerator
+ 2x PRU 32-bit microcontrollers
+ Ubuntu 13.10, [ARMhf image][2]
+ ROS Hydro
+ OpenCV 2.4.8

### Tests Run
1. OpenCV 2.4.8 - standard cross compilation
2. OpenCV 2.4.8 - cross compilation with NEON and VFP support

### Results Summary
1. 2.98 FPS
2. 3.06 FPS

HardKernel ODroid XU
--------------------

### Specification
+ Exynos5 Octa Cortex™-A15 1.6Ghz quad core and Cortex™-A7 quad core CPUs
+ PowerVR SGX544MP3 GPU (OpenGL ES 2.0, OpenGL ES 1.1 and OpenCL 1.1 EP)
+ 2Gbyte LPDDR3 RAM PoP
+ USB 3.0 Host x 1, USB 3.0 OTG x 1, USB 2.0 Host x 4
+ HDMI 1.4a output Type-D connector
+ eMMC 4.5 Flash Storage
+ Ubuntu 13.09, [Hardkernel image][3]
+ ROS Hydro
+ OpenCV 2.4.8

### Tests Run
1. OpenCV 2.4.8 - standard cross compilation
2. OpenCV 2.4.8 - cross compilation with NEON and VFP support
3. OpenCV 2.4.8 - cross compilation with NEON, VFP and TBB support

### Results Summary
1. 8.87 FPS
2. 9.42 FPS
3. 9.46 FPS


Desktop Computer
----------------
### Specification
+ Intel Core i5 2500K CPU
+ 8 GB DDR3 RAM
+ ATI Radeon HD 6950, 2 GB
+ Linux Mint 16 (= Ubuntu 13.10)
+ ROS Hydro
+ OpenCV 2.4.8

### Tests Run
1. OpenCV 2.4.8 - [standard compilation][4] with TBB support enabled

### Results Summary
1. 30.01 FPS


[1] http://docs.opencv.org/doc/tutorials/introduction/crosscompilation/arm_crosscompile_with_cmake.html
[2] http://www.armhf.com/
[3] http://odroid.in/Ubuntu_Server_XU/
[4] https://help.ubuntu.com/community/OpenCV