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
#include "Profiling.h"

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int16.h"
#include <pthread.h>

#include <sstream>

using namespace std;

const int NUM_THREADS = 3;
pthread_t threads [NUM_THREADS];
bool thread_busy [NUM_THREADS];
Mat thread_frames [NUM_THREADS];
int thread_frame_ids [NUM_THREADS];
pthread_cond_t thread_cond[NUM_THREADS] = {PTHREAD_COND_INITIALIZER};
pthread_mutex_t thread_cond_mutexes[NUM_THREADS] = {PTHREAD_MUTEX_INITIALIZER};
pthread_mutex_t thread_busy_mutex = PTHREAD_MUTEX_INITIALIZER;
bool work = true;

//Profiling variables
int volatile  framesProcessed = 0;
double captureTime = 0 , detectCornersTimes[NUM_THREADS] = {0}, calibrateImagePointsTimes[NUM_THREADS] = {0}, estimatePoseTimes[NUM_THREADS] = {0};
double detectCornersSubTimes[NUM_THREADS][8] = {0};

ros::Publisher frameNumberPub;
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

int findOption( int argc, char **argv, const char *option )
{
    for( int i = 1; i < argc; i++ )
    if( strcmp( argv[i], option ) == 0 )
    return i;
    return -1;
}

int readInt( int argc, char **argv, const char *option, int default_value )
{
    int iplace = findOption( argc, argv, option );
    if( iplace >= 0 && iplace < argc-1 )
    return atoi( argv[iplace+1] );
    return default_value;
}

void setThreadBusy (int thread_id) {
    pthread_mutex_lock ( &thread_busy_mutex );
    thread_busy[thread_id] = true;
    pthread_mutex_unlock ( &thread_busy_mutex );
}

void setThreadIdle (int thread_id) {
    pthread_mutex_lock ( &thread_busy_mutex );
    thread_busy[thread_id] = false;
    framesProcessed++;
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
    double lastTime = 0;
    while(do_work()) {
        pthread_cond_wait(&thread_cond[thread_id], &thread_cond_mutexes[thread_id]);
        setThreadBusy (thread_id);
        Mat frame = thread_frames[thread_id];
        Mat_<double> imagePts;
        Mat_<double> simplePose;
        lastTime = readTimer();
        imagePts = detectCorners(frame, &detectCornersSubTimes[thread_id][0]);
        lastTime = getDifferenceAndIncrement(lastTime, &detectCornersTimes[thread_id]);
        std_msgs::Int16 frame_id;
        frame_id.data = thread_frame_ids[thread_id];
        frameNumberPub.publish(frame_id);
        bool success = (imagePts.rows > 0);
        if(success) {
            imagePts = calibrateImagePoints(imagePts);
            lastTime = getDifferenceAndIncrement(lastTime, &calibrateImagePointsTimes[thread_id]);
            std_msgs::Float64MultiArray cornersMsg = makeCornersMsg(imagePts);
            cornersPub.publish(cornersMsg);
            simplePose = estimatePose(imagePts);
            lastTime = getDifferenceAndIncrement(lastTime, &estimatePoseTimes[thread_id]);
            std_msgs::Float64MultiArray simplePoseMsg = \
                makeSimplePoseMsg(simplePose);
            simplePosePub.publish(simplePoseMsg);
            ROS_INFO("Tick.");
        } else {
            // Don't publish anything this tick.
            // ROS_INFO("Could not detect all corners!");
        }

        frame.release();
        setThreadIdle (thread_id);
    }
    pthread_exit(NULL);   
}

int main(int argc, char **argv)
{
    int loopFrequency = readInt ( argc, argv, "-l", -1);

    ros::init(argc, argv, "PoseEstimator");
    ros::NodeHandle node;
    int const queueSize = 1000;
    ros::Rate  loopRate (loopFrequency); 

    // "corners" message: [x1, y1, ..., x24, y24] -- contains the image
    // coordinates of the 24 corners, in the order described in the paper
    cornersPub = \
        node.advertise<std_msgs::Float64MultiArray>("corners", queueSize);

    // "simplePose" message: [x, y, z, yaw] -- contains the estimate
    // of the camera pose w.r.t. the landing pad
    simplePosePub = \
        node.advertise<std_msgs::Float64MultiArray>("simplePose", queueSize);

    frameNumberPub = node.advertise<std_msgs::Int16>("frameNumbers", queueSize);

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

    double startTime = readTimer();
    double totalTime = startTime;
    double lastLoopTime = 0;
    Mat frame;

    while(ros::ok()) {
        lastLoopTime = readTimer();    
        capture >> frame;
        lastLoopTime = getDifferenceAndIncrement(lastLoopTime, &captureTime);

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
            thread_frame_ids[idle_thread] = framesProcessed;
            pthread_cond_signal(&thread_cond[idle_thread]);
            ROS_INFO("Frame despatched to thread %d", idle_thread);
        } else {
            ROS_INFO("No free threads");
            frame.release();
        } 

        if (framesProcessed >= 20) {
            totalTime = readTimer() - totalTime;
            double detectCornersTime = 0, calibrateImagePointsTime = 0, estimatePoseTime = 0,
            medianBlur = 0, canny = 0, findContours = 0, approx1 = 0, getIndex = 0, approx2 = 0,
            labelPolygons = 0, labelCorners = 0;

            for (int t = 0; t < NUM_THREADS; ++t) {
                detectCornersTime += detectCornersTimes[t];
                calibrateImagePointsTime += calibrateImagePointsTimes[t];
                estimatePoseTime += estimatePoseTimes[t];
                medianBlur += detectCornersSubTimes[t][0];
                canny += detectCornersSubTimes[t][1];
                findContours += detectCornersSubTimes[t][2];
                approx1 += detectCornersSubTimes[t][3];
                getIndex += detectCornersSubTimes[t][4];
                approx2 += detectCornersSubTimes[t][5];
                labelPolygons += detectCornersSubTimes[t][6];
                labelCorners += detectCornersSubTimes[t][7];
            }

            ROS_INFO("frames\tseconds\tfps\tcapture\tdetectCorners\tcalibrateImagePoints\testimatePose");
            ROS_INFO("%d\t%g\t%f\t%f\t%f\t%f\t%f",framesProcessed, totalTime, (framesProcessed/totalTime), captureTime, detectCornersTime, calibrateImagePointsTime, estimatePoseTime);
            ROS_INFO("medianBlur\tCanny\tfindContours\tapproxPolydp1\tgetIndexOfOuterSquare\tapproxPolyDP2\tlabelPolygons\tlabelCorners");
            ROS_INFO("%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f", 
                medianBlur, canny, findContours, approx1, getIndex, approx2, 
                labelPolygons,labelCorners);
            totalTime  = readTimer();

            framesProcessed = captureTime = 0;

            for (int t = 0; t < NUM_THREADS; ++t) {
                detectCornersTimes[t] = 0;
                calibrateImagePointsTimes[t] = 0;
                estimatePoseTimes[t] = 0;
                for (int i = 0; i<8; ++i)
                    detectCornersSubTimes[t][i] = 0;
            }
        }
        ros::spinOnce();
        if (loopFrequency)
            loopRate.sleep();
    }
    ROS_INFO("Master stopping work");
    stop_work();
    return 0;
}
