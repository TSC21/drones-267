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

bool controllerActive = false;

bool isControllerActive() {
    return controllerActive;
}

bool updateControllerActive(int controlChannelValue) {
    bool oldControllerActive = controllerActive;

    //Add 10 either side to account for inaccuracy in signal
    if (controlChannelValue <= (MODE_LOITER + 10) && \
    	controlChannelValue >= (MODE_LOITER - 10)) 
        controllerActive = true;
    else
        controllerActive = false;

    return (oldControllerActive == controllerActive);
}

States currentState;

States getState () {
    return currentState;
}

bool setState (States newState) {
    bool changed = true;
    if (newState == currentState) {
        changed = false;
    }
    else {
        currentState = newState;
    }
    return changed;
}

std::vector<roscopter::RC> getStateAction() {
	std::vector<roscopter::RC> controlMsgs;
    switch (currentState) {
        case FLYING:
        	controlMsgs.push_back(getManualControlMsg());
            break;
        case HOVERING:
        	controlMsgs.push_back(getTranslateControlMsg());
        	break;
    }
    return controlMsgs;
}

std::vector<roscopter::RC> getNeutralAction() {
	std::vector<roscopter::RC> controlMsgs;
	controlMsgs.push_back(getNeutralControlMsg());
    return controlMsgs;
}
