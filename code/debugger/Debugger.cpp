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


#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/String.h"

using namespace std;

int main(int argc, char **argv) 
{
	ros::init(argc, argv, "Debugger");
    ros::NodeHandle node;
    int const queueSize = 1000;
    ros::Rate loopRate(4);  // publish messages at 4 Hz

    //ros::Subscriber sub = node.subscribe("control", 1000, controlCallback);

	//Debugger posts output to command line
	//Log info messages too?

    ros::spin();

    return 0;
}

/* Messages of interest-
* 
* ==pose_estimator==
* corners
* simplePose
*
* ==lander==
* control
* controlOutput
* controlInput
* states
*
* ==roscopter==
* rc
* send_rc
*/
