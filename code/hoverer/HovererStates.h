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

#ifndef LANDER_STATES_H
#define LANDER_STATES_H

#include <vector>
#include "roscopter/RC.h"


/**
* Enum of all possible Lander states:
* FLYING (stabilise)
* LAND_HIGH (loiter)
* LAND_LOW (loiter)
* POWER_OFF (loiter)
*/
enum States {
    FLYING,
    HOVERING
};

/**
* Gets the current active state of the Hoverer
*
* Returns:
* controller = true if the Hoverer is currently active
*/
bool isControllerActive();

/**
* Updates the current active state of the Hoverer.
*
* Inputs:
* controlChannelValue = the current integer value of the
* RC control channel.
*
* Returns:
* changed = true if the Hoverer state has changed
*/
bool updateControllerActive(int controlChannelValue);

/**	
* Get the current state.
*
* Returns:
* currentState = the current state
*/
States getState ();

/**
* Set the current state to a new state.
*
* Inputs:
* newState = new state
*
* Returns:
* changed = true if the new state is different
*/
bool setState(States newState);

/**
* Gets the appropriate control input for
* this state, at this time.
*
* Returns:
* controlMsg = roscopter::RC control input
*/
std::vector<roscopter::RC> getStateAction ();


/**
* Returns a neutral input that will reset values 
* to neutral to maintain straight and level flight.
*
* Returns:
* controlMsg = roscopter::RC control input
*/
std::vector<roscopter::RC> getNeutralAction ();

#endif
