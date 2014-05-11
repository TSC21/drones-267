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


#include "QuadcopterState.h"

const float NO_CLIMB_RATE = 0.01;
const float GROUND_ALT = 0.1; 
const float MIN_FIELD_OF_VIEW_ALT = 0.75;
const float STABILITY_LIMIT = 0.2;
const float PI_F=3.14159265358979f;
const float CAMERA_ROTATION = 0; // Camera is pointing forwards

QuadcopterState latestState = QuadcopterState();
QuadcopterPose latestPose = QuadcopterPose();
QuadcopterPose startingPose = QuadcopterPose();

void updateAttitude(const roscopter::Attitude::ConstPtr& attitudeMsg) {
	ROS_DEBUG("Updated attitude");
	latestState.roll = attitudeMsg->roll;
	latestState.pitch = attitudeMsg->pitch;
	latestState.yaw = attitudeMsg->yaw;
}

void updateTelemetry (const roscopter::VFR_HUD::ConstPtr& hudMsg) {
	ROS_DEBUG("Updated telemetry");
 	latestState.airspeed  = hudMsg->airspeed;
	latestState.groundspeed = hudMsg->groundspeed;
	latestState.heading = hudMsg->heading;
	latestState.throttle = hudMsg->throttle;
	latestState.alt = hudMsg->alt;
	latestState.climb = hudMsg->climb;
}

double normaliseYaw(double yaw) {
	double normalisedYaw = yaw - CAMERA_ROTATION;
	if (normalisedYaw < (-1 * PI_F)) {
		normalisedYaw += 2 * PI_F;
	}
	return normalisedYaw;
}

void updatePose (const std_msgs::Float64MultiArray::ConstPtr& poseMsg) {
	//[x, y, z, yaw]
	ROS_DEBUG("Updated pose");
	latestPose.x = poseMsg->data[0];
	latestPose.y = poseMsg->data[1];
	latestPose.z = poseMsg->data[2];
	latestPose.yaw = normaliseYaw(poseMsg->data[3]);
}

void setStartingPose () {
	startingPose = latestPose;
}

QuadcopterState getQuadcopterState () {
	return latestState;
}

QuadcopterPose getQuadcopterPose () {
	return latestPose;
}

QuadcopterPose getStartingPose () {
	return startingPose;
}