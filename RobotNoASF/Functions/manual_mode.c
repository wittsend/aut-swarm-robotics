/*
* manual_mode.c
*
* Author : Adam Parlane
* Created: 1/8/2017
*
* Project Repository: https://github.com/wittsend/aut-swarm-robotics
*
* Contains the definitions and functions required for manual mode
*
* More Info:
* Atmel SAM 4N Processor Datasheet:http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
*
* Functions:
* manualControl(message)
*
*/

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "../robot_setup.h"

#include "../Interfaces/xbee_driver.h"
#include "../Interfaces/motor_driver.h"

#include "manual_mode.h"
#include "motion_functions.h"
#include "comm_functions.h"		//Debug strings

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
/*
* Function:
* void manualControl(struct MessageInfo message)
*
* Runs the manual movement controls from the GUI
* These are:
*			move straight (N, NE, E, SE, S, SW, W, NW)
*			rotate (MC_CW, CCW)
*			stop
* Movements also have an assigned speed
*
* Inputs:
* RobotGlobalStructure *sys
*   Pointer to the global system data structure where the last message received by the Xbee can be
*   retrieved
*
* Returns:
* none
*
* Implementation:
* uses comConvertData(message, receivedTestData);
* to fetch the received data information (speed and direction)
*
*
* Improvements:
* TODO: Adam Comment this -AP
*
*/
void manualControl(RobotGlobalStructure *sys)
{
	switch(sys->comms.xbeeMessageType)
	{
		case MC_STOP:
			mfStopRobot(sys);
			commSendDebugFloat("STOP", (float) (sys->timeStamp)/1000, sys);
			sys->states.mainf = M_IDLE;
			break;

		case MC_MOVE:
			{
			int16_t robotRelativeHeading = (sys->comms.xbeeMessageData[0] << 8) + (sys->comms.xbeeMessageData[1]);
			mfAdvancedMove(robotRelativeHeading + sys->pos.facing, sys->pos.facing, sys->comms.xbeeMessageData[2], 100, sys);
			sys->pos.targetHeading = robotRelativeHeading;
			sys->pos.targetSpeed = sys->comms.xbeeMessageData[2];
			commSendDebugFloat("MOVE", (float) (sys->timeStamp)/1000, sys);
			}
			break;
		
		case MC_CCW:
			//MC_CW is reverse so invert speed
			moveRobot(0, -(int8_t)sys->comms.xbeeMessageData[0], 100);
			break;
		
		case MC_CW:
			//CCW is forward so no need to invert speed
			moveRobot(0, sys->comms.xbeeMessageData[0], 100);
			break;
	}
}