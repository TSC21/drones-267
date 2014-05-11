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


#include "Controller.h"
#include "HovererStates.h"
#include "QuadcopterState.h"

#include <vector>
#include "std_msgs/Int32MultiArray.h"


// values from radio calibration in MissionPlanner
// 1: aileron, neutral = 1527, low = 1128, high = 1925
// 2: elevator, neutral = 1528, low = 1921, high = 1124
// 3: throttle, neutral ?, low = 1124, high = 1927
// 4: yaw, neutral = 1530, low = 1131, high = 1928
// 5: mode, low = 1128, med = 1528, high = 1928

enum Aileron {
	AILERON_LOW = 1128,
	AILERON_HIGH = 1925,
	AILERON_RANGE = 797
};

enum Elevator {
	ELEVATOR_LOW = 1124,
	ELEVATOR_HIGH = 1921,
	ELEVATOR_RANGE = 797 
};

enum Throttle {
	THROTTLE_LOW = 1124,
	THROTTLE_HIGH = 1927,
	THROTTLE_RANGE = 803
};

enum Yaw {
	YAW_LOW = 1128,
	YAW_HIGH = 1928,
	YAW_RANGE = 800
};

const int NEUTRAL = 1528;
int AILERON_NEUTRAL = NEUTRAL;
int ELEVATOR_NEUTRAL = NEUTRAL;
int THROTTLE_NEUTRAL = NEUTRAL;
int YAW_NEUTRAL = NEUTRAL;

const int LATERAL_GAIN = 0.5;
const int VERTICAL_GAIN = 0.2;

const float PI_F=3.14159265358979f;
const float YAW_CORRECTION = 0.08; // ~= 5 degrees 

// Global, set by cmdline param
// Max error for x,y dimensions in mm
int MAX_DISPLACEMENT_ERROR;

int getMaxDisplacementError()
{
    return MAX_DISPLACEMENT_ERROR;
}

void setMaxDisplacementError(int err)
{
    MAX_DISPLACEMENT_ERROR = err;
}

roscopter::RC buildRCMsg(int aileron, int elevator, int throttle, int yaw) {
	roscopter::RC msg;
	msg.channel[0] = aileron;
	msg.channel[1] = elevator;
	msg.channel[2] = throttle;
	msg.channel[3] = yaw;

	//We never override channel 4, this will let us recover to manual control.
	msg.channel[4] = 0;

	//Channels 5-7 are not in use, so we set them = 0.
	msg.channel[5] = 0;
	msg.channel[6] = 0;
	msg.channel[7] = 0;

	return msg;
}

roscopter::RC getTranslateControlMsg () {
	if (getQuadcopterPose().yaw > YAW_CORRECTION || getQuadcopterPose().yaw < (-1*YAW_CORRECTION)) {
		return getRotationControlMsg();
	}

	// Assume we are always pointing forwards and therefore
	// can map x and y onto aileron and elevator inputs

	double x_error = getStartingPose().x - getQuadcopterPose().x; //mm
	double y_error = getStartingPose().y - getQuadcopterPose().y; //mm
	double z_error = getStartingPose().z - getQuadcopterPose().z; //mm

	float aileron_gain = -1 * (x_error * LATERAL_GAIN);
	float elevator_gain = -1 * (y_error * LATERAL_GAIN);
	float throttle_gain = -1 * (z_error * VERTICAL_GAIN);

	int aileron = (int) ((aileron_gain * AILERON_RANGE) / 2) + AILERON_NEUTRAL;
	int elevator = (int) ((elevator_gain * ELEVATOR_RANGE) / 2) + ELEVATOR_NEUTRAL;
	int throttle = (int) ((throttle_gain * THROTTLE_RANGE) / 2) + THROTTLE_NEUTRAL;

	return buildRCMsg(aileron, elevator, throttle, YAW_NEUTRAL);
}

roscopter::RC getManualControlMsg () {
	return buildRCMsg(0,0,0,0);
}

void updateRC(const roscopter::RC::ConstPtr& rcMsg) {
	if (!isControllerActive()) {
		AILERON_NEUTRAL = rcMsg->channel[0];
		ELEVATOR_NEUTRAL = rcMsg->channel[1];
		THROTTLE_NEUTRAL = rcMsg->channel[2];
		YAW_NEUTRAL = rcMsg->channel[3];
	}
}

