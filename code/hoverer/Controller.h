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

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "roscopter/RC.h"

/* User defined in MissionPlanner */
enum Mode {
	MODE_RTL = 1128,
	MODE_LOITER = 1528,
	MODE_STABILISE = 1928
};

int getMaxDisplacementError();
void setMaxDisplacementError(int value);

/**
* Responsible for generating all control input messages
*/
roscopter::RC getTranslateControlMsg ();

/**
* Creates a message that will return the drone to manual control.
*/
roscopter::RC getManualControlMsg ();

void updateRC(const roscopter::RC::ConstPtr& rcMsg) ;

#endif
