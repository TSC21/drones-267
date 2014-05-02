/*
*  drones-267, automated landing using pose estimation
*  Copyright (C) {2013}  {Constantin Berzan, Nahush Bhanage, Sunil Shah}
*  
*  https://github.com/ssk2/drones-267
*
*  This program is free software: you can redistribute it and/or modify
*  it under the terms of the GNU General Public License as published by
*  the Free Software Foundation, either version 3 of the License, or
*  (at your option) any later version.
* 
*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
* 
*  You should have received a copy of the GNU General Public License
*  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "Corners.h"
#include "Geometry.h"

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include <pthread.h>

#include <sstream>
using namespace std;

const int NUM_THREADS = 6;
pthread_t threads [NUM_THREADS];
bool thread_busy [NUM_THREADS];
Mat thread_frames [NUM_THREADS];
pthread_cond_t thread_cond[NUM_THREADS] = {PTHREAD_COND_INITIALIZER};
pthread_mutex_t thread_cond_mutexes[NUM_THREADS] = {PTHREAD_MUTEX_INITIALIZER};
pthread_mutex_t thread_busy_mutex = PTHREAD_MUTEX_INITIALIZER;
bool work = true;

ros::Publisher cornersPub;
ros::Publisher simplePosePub;

std_msgs::Float64MultiArray makeCornersMsg(Mat_<double> const imagePts)
{
    assert(imagePts.rows == 24);
    assert(imagePts.cols == 2);
    std_msgs::Float64MultiArray cornersMsg;
    for(int i = 0; i < 24; i++) {
        cornersMsg.data.push_back(imagePts(i, 0));
        cornersMsg.data.push_back(imagePts(i, 1));
    }
    return cornersMsg;
}

std_msgs::Float64MultiArray makeSimplePoseMsg(Mat_<double> const simplePose)
{
    std_msgs::Float64MultiArray simplePoseMsg;
    simplePoseMsg.data.push_back(simplePose(0));
    simplePoseMsg.data.push_back(simplePose(1));
    simplePoseMsg.data.push_back(simplePose(2));
    simplePoseMsg.data.push_back(simplePose(3));
    return simplePoseMsg;
}

void setThreadBusy (int thread_id) {
    pthread_mutex_lock ( &thread_busy_mutex );
    thread_busy[thread_id] = true;
    pthread_mutex_unlock ( &thread_busy_mutex );
}

void setThreadIdle (int thread_id) {
    pthread_mutex_lock ( &thread_busy_mutex );
    thread_busy[thread_id] = false;
    pthread_mutex_unlock ( &thread_busy_mutex );
}

bool do_work()
{
    pthread_mutex_lock ( &thread_busy_mutex );
    bool do_work = work;
    pthread_mutex_unlock ( &thread_busy_mutex );
    return do_work;
}

bool stop_work ()
{
    pthread_mutex_lock ( &thread_busy_mutex );
    work = false;
    pthread_mutex_unlock ( &thread_busy_mutex );

    for (int t=0; t<NUM_THREADS; ++t) {
        ROS_INFO("Waiting for thread %d to join", t);
        pthread_join ( threads[t], NULL );
    }
}

void *processFrame (void *arg) {
    int thread_id = *(int *)arg;
    while(do_work()) {
        pthread_cond_wait(&thread_cond[thread_id], &thread_cond_mutexes[thread_id]);
        setThreadBusy (thread_id);
        Mat frame = thread_frames[thread_id];
        Mat_<double> imagePts;
        Mat_<double> simplePose;
        imagePts = detectCorners(frame);
        bool success = (imagePts.rows > 0);
        if(success) {
            imagePts = calibrateImagePoints(imagePts);
            std_msgs::Float64MultiArray cornersMsg = makeCornersMsg(imagePts);
            cornersPub.publish(cornersMsg);
            simplePose = estimatePose(imagePts);
            std_msgs::Float64MultiArray simplePoseMsg = \
                makeSimplePoseMsg(simplePose);
            simplePosePub.publish(simplePoseMsg);
            ROS_INFO("Tick.");
        } else {
            // Don't publish anything this tick.
            ROS_INFO("Could not detect all corners!");
        }

        frame.release();
        setThreadIdle (thread_id);
    }
    ROS_INFO("Thread %d exiting.", thread_id);
    pthread_exit(NULL);
    
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "PoseEstimator");
    ros::NodeHandle node;
    int const queueSize = 1000;
    ros::Rate loopRate(4);  // publish messages at 4 Hz

    // "corners" message: [x1, y1, ..., x24, y24] -- contains the image
    // coordinates of the 24 corners, in the order described in the paper
    cornersPub = \
        node.advertise<std_msgs::Float64MultiArray>("corners", queueSize);

    // "simplePose" message: [x, y, z, yaw] -- contains the estimate
    // of the camera pose w.r.t. the landing pad
    simplePosePub = \
        node.advertise<std_msgs::Float64MultiArray>("simplePose", queueSize);

    VideoCapture capture(0);
    if(!capture.isOpened()) {
        cerr << "Device inaccessible. Damn!" << endl;
        return 1;
    }

    // Set exposure and focus manually.
    // Seems to take effect after several frames.
    Mat temp_frame;
    capture >> temp_frame;
    temp_frame.release();
    system("v4l2-ctl -d 0 -c exposure_auto=1");
    system("v4l2-ctl -d 0 -c exposure_absolute=180");
    system("v4l2-ctl -d 0 -c focus_auto=0");
    system("v4l2-ctl -d 0 -c focus_absolute=0");

    int thread_ids[NUM_THREADS];

    // Create threads    
    for (int t=0; t<NUM_THREADS; ++t) {
        thread_ids[t] = t;
        pthread_create ( &threads[t], NULL, processFrame, (void *) &thread_ids[t]);
    }

    Mat frame;

    while(ros::ok()) {

        capture >> frame;

        // Despatch frame to an available thread
        pthread_mutex_lock ( &thread_busy_mutex );
        int idle_thread = -1;

        for (int t = 0; t < NUM_THREADS; ++t) {
            if (!thread_busy[t]) {
                idle_thread = t;
                break;
            }
        }

        pthread_mutex_unlock ( &thread_busy_mutex );

        if (idle_thread > -1) {
            thread_frames[idle_thread] = frame;    
            pthread_cond_signal(&thread_cond[idle_thread]);
            ROS_INFO("Frame despatched to thread %d", idle_thread);  
        } else {
            ROS_INFO("No free threads");
            frame.release();
        }

        ros::spinOnce();
    }
    ROS_INFO("Master stopping work");
    stop_work();
    return 0;
}
