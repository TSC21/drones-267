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


#include <iostream>
#include <map>
#include <string>
using namespace std;

#include "Controller.h"
#include "HovererStates.h"
#include "QuadcopterState.h"

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"

#include "roscopter/Attitude.h"
#include "roscopter/RC.h"
#include "roscopter/VFR_HUD.h"

#include <boost/program_options.hpp>
namespace po = boost::program_options;

ros::Publisher rcPub;

map<States, string> stateName;

// Starting state; set by command-line param.
States START_STATE;

const int MAX_CONTROL_CYCLES = 2;
int cyclesSinceControlInput = 0;

void publishControlMsgs(std::vector<roscopter::RC> controlMsgs) {
	for (int i=0; i<controlMsgs.size(); i++) {
		roscopter::RC controlMsg = controlMsgs[i];
		ROS_INFO("%d %d %d %d", controlMsg.channel[0], controlMsg.channel[1], controlMsg.channel[2], controlMsg.channel[3]); 
		rcPub.publish(controlMsg);	
	}
}

void performStateAction() {
	ROS_INFO("Sending control message");
	publishControlMsgs(getStateAction());
}

void setStateAndPerformAction(States newState) {
	setState(newState);
	performStateAction();
}


void attitudeCallback(const roscopter::Attitude::ConstPtr& attitudeMsg) {
	updateAttitude(attitudeMsg);
}

/**
* State transitions happen when we receive one of the following:
* New quadcopter state estimate
* New pose estimate
* New RC input
*/

void hudCallback(const roscopter::VFR_HUD::ConstPtr& hudMsg) {
	updateTelemetry(hudMsg);
}

void poseCallback(const std_msgs::Float64MultiArray::ConstPtr& poseMsg) {
	updatePose(poseMsg);

	if (isControllerActive()) {		
		if (getState() == HOVERING) {
			ROS_INFO("Performing action: HOVERING");
			performStateAction();
		} else {
			ROS_INFO("Setting state and performing action: HOVERING");
			// Capture hover values
			setStartingPose();
			setStateAndPerformAction(HOVERING);
		}
	} 
	// Not active, don't do anything
}

void rcCallback(const roscopter::RC::ConstPtr& rcMsg) {
	updateRC(rcMsg);

	if (updateControllerActive(rcMsg->channel[4])) {  // CHECK CHANNEL
		if (isControllerActive()) {
			// Revert to manual control
			ROS_INFO("Controller now inactive");
			ROS_INFO("Setting state and performing action: FLYING");
			setStateAndPerformAction(FLYING);
		} else {
			ROS_INFO("Controller now active");
		}
	}

	// Else no change
}

// Parse command line and update global params.
void parseCommandLine(int argc, char **argv)
{
    po::options_description desc("Allowed options");
    desc.add_options()
        ("max-displ", po::value<int>(), "max displacement error")
        ("start-state", po::value<string>(), "start state")
    ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    // MAX_DISPLACEMENT_ERROR
    if (vm.count("max-displ")) {
        setMaxDisplacementError(vm["max-displ"].as<int>());
    } else {
        setMaxDisplacementError(1000);  // default
    }
    cout << "MAX_DISPLACEMENT_ERROR: " << getMaxDisplacementError() << endl;

    // START_STATE
    // FIXME what's a good place to initialize this?
    stateName[FLYING] = "FLYING";
    stateName[HOVERING] = "HOVERING";
    if (vm.count("start-state")) {
        string value = vm["start-state"].as<string>();
        // HACK: iterate enum values; assumes HOVERING is the last one.
        int i;
        for(i = 0; i <= HOVERING; i++) {
            if(stateName[(States)i] == value) {
                START_STATE = (States)i;
                break;
            }
        }
        if(i == HOVERING + 1) {
            cerr << "Unrecognized start state." << endl;
            exit(1);
        }
    } else {
        START_STATE = HOVERING;  // default
    }
    cout << "START_STATE: " << stateName[START_STATE] << endl;
}

int main(int argc, char **argv) {
    parseCommandLine(argc, argv);

	ros::init(argc, argv, "Hoverer");
    ros::NodeHandle node;
    int const queueSize = 1000;

	ROS_INFO("Setting up subscriptions");
    // Subscribe to pose estimates
    ros::Subscriber simplePoseSub = \
    	node.subscribe("simplePose", queueSize, poseCallback);

    // Subscribe to RC inputs
    ros::Subscriber rcSub = \
    	node.subscribe("rc", queueSize, rcCallback);

    // Subscribe to quadcopter state
    ros::Subscriber hudSub = \
    	node.subscribe("vfr_hud", queueSize, hudCallback);

    ros::Subscriber attitudeSub = \
    	node.subscribe("attitude", queueSize, attitudeCallback);

    // Set up publisher for RC output
    rcPub = \
    	node.advertise<roscopter::RC>("send_rc", queueSize);

    ros::spin();

    return 0;
}
