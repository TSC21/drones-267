Autonomous Multi-Rotor Landing Using Vision
===========================================

This project is the amalgamation of multiple class projects and a Master's capstone project:
+ CS280: Computer Vision (Fall 2013)
+ CS287: Advanced Robotics (Fall 2013)
+ CS267: Applications of Parallel Computers (Spring 2014)

We're aligned with the Cyber-Physical Cloud Computing Lab at UC Berkeley (http://cpcc.berkeley.edu).

The goal is to develop a robust automated landing controller that uses a landing station of a known pattern to very accurately land a multi-rotor unmanned aerial vehicle. We aim to use open source software, to run on open hardware and to run at high performance.

Our approach is based on the 2001 paper "A vision system for landing an unmanned aerial vehicle." by Sharp, Courtney S., Omid Shakernia, and S. Shankar Sastry. 

Hardware Set Up
---------------
### Autopilot
We've tested this against an APM 2.6 running Ardupilot. We assume that the computer is connected to the APM via USB. Note that this means the 3DR telemetry link will be disabled since both USB and telemetry cannot operate at the same time.

### Embedded Board
We've used this on a [BeagleBone Black][1] running a [12.04 ARMhf image][2]. We've also used this on a [HardKernel Odroid XU][3] using the [HardKernel 13.09 Ubuntu Server image][4].

Software Set Up
---------------
You'll need to build this code from source. It runs against ROS Hydro and OpenCV.

If setting up on an embedded board, you can install ROS and OpenCV by hand. (If using an ARM board, you'd be advised to [cross compile OpenCV][5]). 

For development purposes, you can install vagrant and VirtualBox, clone this repository and run `vagrant up` in the root of it. Vagrant works well for testing compilation but it's performance with hardware is less than satisfactory.

(If you want to test this with a webcam/ArduPilot, you will need to call `vagrant halt`, open the VirtualBox settings for the newly created virtual machine and add USB device filters for the relevant devices.)

### Pre-Requisites
This project depends on [roscopter][6] being built and available.

### Build
vagrant will automatically symlink the repository folder into ~/catkin_ws/src/drones-267.

If you're not using vagrant, be sure to clone this repository in ~/catkin_ws/src.

```bash
cd ~/catkin_ws
catkin_make
```


Usage
-----
The state controller works by listening to the mode switch (typically channel 6). 

More details to come.

Performance
-----------
A goal of this project was to have this operate at greater than 5 frames per second to allow for adequate control.

More data to come.

Contributors
------------
Thanks to @cberzan, @nbhanage and @hoanghw!

[1]: http://beagleboard.org/Products/BeagleBone%20Black
[2]: http://www.armhf.com/download/
[3]: http://www.hardkernel.com/main/products/prdt_info.php?g_code=G137510300620
[4]: http://odroid.in/Ubuntu_Server_XU/
[5]: http://docs.opencv.org/doc/tutorials/introduction/crosscompilation/arm_crosscompile_with_cmake.html
[6]: https://github.com/ssk2/roscopter
