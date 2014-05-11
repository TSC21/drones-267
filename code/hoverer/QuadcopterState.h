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


#ifndef QUADCOPTER_STATE_H
#define QUADCOPTER_STATE_H

#include "std_msgs/Float64MultiArray.h"

#include "ros/ros.h"
#include "roscopter/Attitude.h"
#include "roscopter/VFR_HUD.h"

struct QuadcopterState {
    double roll;
    double pitch;
    double yaw;
    double airspeed;
    double groundspeed;
    int heading;
    int throttle;
    double alt;
    double climb;
};


struct QuadcopterPose {
    double x;
    double y;
    double z;
    double yaw;
};

/**
* Updates the latest attitude values from the autopilot
*
* Input:
* attitudeMsg = Attitude message posted by roscopter
*/
void updateAttitude(const roscopter::Attitude::ConstPtr& attitudeMsg);

/**
* Updates the latest telemetry values from the autopilot
*
* Input:
* hudMsg = VFR_HUD message posted by roscopter
*/
void updateTelemetry (const roscopter::VFR_HUD::ConstPtr& hudMsg);

/**
* Updates the latest pose values from the pose_estimator
*
* Input:
* poseMsg = simplePose message posted by pose_estimator
*/
void updatePose (const std_msgs::Float64MultiArray::ConstPtr& poseMsg);


/**
* Sets the starting pose to the currently known pose values
*/
void setStartingPose();

/**
* Evaluates whether the quadcopter is on the ground or not.
*
* Output:
* boolean value, true if quadcopter is at ground level.
*/
QuadcopterState getQuadcopterState();


/**
* Evaluates whether the quadcopter is on the ground or not.
*
* Output:
* boolean value, true if quadcopter is at ground level.
*/
QuadcopterPose getQuadcopterPose();

#endif
