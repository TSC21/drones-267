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

const int NUM_THREADS = 2;
int idle_threads = NUM_THREADS;
bool thread_busy [NUM_THREADS];
Mat thread_frames [NUM_THREADS];
pthread_mutex_t idle_threads_mutex = PTHREAD_MUTEX_INITIALIZER;

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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "PoseEstimator");
    ros::NodeHandle node;
    int const queueSize = 1000;
    ros::Rate loopRate(4);  // publish messages at 4 Hz

    // "corners" message: [x1, y1, ..., x24, y24] -- contains the image
    // coordinates of the 24 corners, in the order described in the paper
    ros::Publisher cornersPub = \
        node.advertise<std_msgs::Float64MultiArray>("corners", queueSize);

    // "simplePose" message: [x, y, z, yaw] -- contains the estimate
    // of the camera pose w.r.t. the landing pad
    ros::Publisher simplePosePub = \
        node.advertise<std_msgs::Float64MultiArray>("simplePose", queueSize);

    VideoCapture capture(0);
    if(!capture.isOpened()) {
        cerr << "Device inaccessible. Damn!" << endl;
        return 1;
    }

    // Set exposure and focus manually.
    // Seems to take effect after several frames.
    capture >> frame;
    system("v4l2-ctl -d 0 -c exposure_auto=1");
    system("v4l2-ctl -d 0 -c exposure_absolute=180");
    system("v4l2-ctl -d 0 -c focus_auto=0");
    system("v4l2-ctl -d 0 -c focus_absolute=0");


    // Create threads
    pthread_t threads [NUM_THREADS];
    
    for (int t=0; t<NUM_THREADS; ++t) {
        pthread_create ( &threads[t], NULL, processFrame, *cornersPub, *simplePosePub, *thread_id );
    }

    while(ros::ok()) {

        Mat frame;
        capture >> frame;
        
        // DESPATCH FRAME TO THREAD IF THERE ARE THREADS AVAILABLE
        pthread_mutex_lock ( &idle_threads_mutex );
        if (idle_threads) {
            int idle_thread = 0;
            while (idle_thread < NUM_THREADS && thread_busy[idle_thread]) {
                idle_thread++;
            }
            thread_frames[idle_thread] = frame;             
        }
        pthread_mutex_unlock ( &idle_threads_mutex );

        ros::spinOnce();
        loopRate.sleep();
    }

    for (int t=0; t<NUM_THREADS; ++t) {
        pthread_join ( &threads[t], NULL );
    }
}


void setThreadBusy (thread_id) {
    pthread_mutex_lock ( &idle_threads_mutex );
    idle_threads--;
    thread_busy[thread_id] = true;
    pthread_mutex_unlock ( &idle_threads_mutex );
}

void setThreadIdle (thread_id) {
    pthread_mutex_lock ( &idle_threads_mutex );
    idle_threads++;
    thread_busy[thread_id] = false;
    pthread_mutex_unlock ( &idle_threads_mutex );
}

void *processFrame (ros::Publisher *cornersPub, ros::Publisher *simplePosePub, int *thread_id) {
    while (true)
    {
        if (thread_frames[thread_id]) {
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

            frame.clear();
            setThreadIdle (thread_id);
        }
    }

        



}
