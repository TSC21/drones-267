#include "Corners.h"
#include "Geometry.h"
#include "Profiling.h"

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"

#include <sstream>
using namespace std;

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

int main(int argc, char **argv)
{
    int loopFrequency = readInt ( argc, argv, "-l", -1);
    int framesProcessed = 0;

    ros::init(argc, argv, "PoseEstimator");
    ros::NodeHandle node;
    int const queueSize = 1000;
    ros::Rate  loopRate (loopFrequency); 

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

    Mat frame;
    Mat_<double> imagePts;
    Mat_<double> simplePose;

    // Set exposure and focus manually.
    // Seems to take effect after several frames.
    capture >> frame;
    system("v4l2-ctl -d 0 -c exposure_auto=1");
    system("v4l2-ctl -d 0 -c exposure_absolute=180");
    system("v4l2-ctl -d 0 -c focus_auto=0");
    system("v4l2-ctl -d 0 -c focus_absolute=0");

    double startTime = readTimer();
    double totalTime = startTime;
    double lastLoopTime = 0, captureTime = 0 , detectCornersTime = 0, calibrateImagePointsTime = 0, estimatePoseTime = 0;
    double detectCornersTimes[8] = {0};

    while(ros::ok()) {
        lastLoopTime = readTimer();
        capture >> frame;
        lastLoopTime = getDifferenceAndIncrement(lastLoopTime, &captureTime);
        imagePts = detectCorners(frame, &detectCornersTimes[0]);
        lastLoopTime = getDifferenceAndIncrement(lastLoopTime, &detectCornersTime);
        framesProcessed++;
        bool success = (imagePts.rows > 0);
        if(success) {
            imagePts = calibrateImagePoints(imagePts);
            lastLoopTime = getDifferenceAndIncrement(lastLoopTime, &calibrateImagePointsTime);
            std_msgs::Float64MultiArray cornersMsg = makeCornersMsg(imagePts);
            cornersPub.publish(cornersMsg);
            simplePose = estimatePose(imagePts);
            lastLoopTime = getDifferenceAndIncrement(lastLoopTime, &estimatePoseTime);
            std_msgs::Float64MultiArray simplePoseMsg = \
                makeSimplePoseMsg(simplePose);
            simplePosePub.publish(simplePoseMsg);
            //ROS_INFO("Tick.");
        } else {
            // Don't publish anything this tick.
            ROS_INFO("Could not detect all corners!");
        }

        ros::spinOnce();
        if (loopFrequency)
            loopRate.sleep();

        if (framesProcessed == 20) {
            totalTime = readTimer() - totalTime;
            ROS_INFO("%d frames processed over %g seconds, %f FPS", framesProcessed, totalTime, (framesProcessed/totalTime));
            ROS_INFO("capture: %f, detectCorners: %f, calibrateImagePoints: %f, estimatePose: %f", captureTime, detectCornersTime, calibrateImagePointsTime, estimatePoseTime);
            ROS_INFO("medianBlur: %f, Canny: %f, findContours: %f, approxPolydp1: %f, getIndexOfOuterSquare: %f, approxPolyDP2: %f, labelPolygons: %f, labelCorners: %f", 
                detectCornersTimes[0], detectCornersTimes[1], detectCornersTimes[2], detectCornersTimes[3], detectCornersTimes[4], detectCornersTimes[5], 
                detectCornersTimes[6], detectCornersTimes[7]);
            totalTime  = readTimer();
            framesProcessed = captureTime = detectCornersTime = calibrateImagePointsTime = estimatePoseTime = 0;
            for (int i = 0; i<8; ++i)
                detectCornersTimes[i] = 0;
        }
    }
}
