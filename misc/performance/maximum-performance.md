
Benchmarking The Maximum Possible Peformance
============================================
Using the script below, it's possible to see how fast a single threaded frame grabbing program can run on an ARM board.
Running without the `-p` flag to enable OpenCV frame decoding, you should see the camera provide frames at about 30 FPS. With the `-p` flag, it'll show the board's maximum capability for grabbing and decompressing frames using OpenCV. It's possible to go faster than this with multi-threading.

Compilation
-----------
Compile the code from [framegrabberCV][http://blog.lemoneerlabs.com/3rdParty/Darling_BBB_30fps_DRAFT.html].

```
gcc -o framegrabberCV framegrabberCV.c `pkg-config --cflags --libs opencv`
```

Setup
-----
Optionally set the performance governor for each of these tests to forgo the effect of CPU scaling:
sudo cpufreq-set -g performance

ODroid
------
## OpenCV processing
```
sunil@odroid-server:~/workspace/framegrabber$ time ./framegrabberCV -f mjpeg -H 480 -W 640 -c 1000 -I 30 -p -o 1
Startup took 0.000000 seconds
Captured 1000 frames and Processed 1000 in 6.930000 seconds
Shutdown took 0.000000 seconds


real    0m33.703s
user    0m5.460s
sys     0m1.480s
```

## No OpenCV processing
```
sunil@odroid-server:~/workspace/framegrabber$ time ./framegrabberCV -f mjpeg -H 480 -W 640 -c 1000 -I 30 -o 1
Startup took 0.010000 seconds
Captured 0 frames and Processed 0 in 0.060000 seconds
Shutdown took 0.000000 seconds


real    0m33.656s
user    0m0.025s
sys     0m0.050s
```

Beaglebone
----------
## OpenCV processing
```
sunil@ubuntu-armhf:~/catkin_ws/src/drones-267/misc/CaptureFPS/build$ time ./framegrabberCV -f mjpeg -H 480 -W 640 -c 1000 -I 30 -p -o 1
Startup took 0.010000 seconds
Captured 1000 frames and Processed 1000 in 18.140000 seconds
Shutdown took 0.000000 seconds


real    0m35.448s
user    0m15.443s
sys     0m2.730s
```

## No OpenCV processing
```
sunil@ubuntu-armhf:~/catkin_ws/src/drones-267/misc/CaptureFPS/build$ time ./framegrabberCV -f mjpeg -H 480 -W 640 -c 1000 -I 30 -o 1
Startup took 0.000000 seconds
Captured 0 frames and Processed 0 in 0.140000 seconds
Shutdown took 0.000000 seconds


real    0m33.643s
user    0m0.021s
sys     0m0.156s
```