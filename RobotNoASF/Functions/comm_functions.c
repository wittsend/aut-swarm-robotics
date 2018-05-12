/*
* comm_functions.c
*
* Author : Adam Parlane/Matthew Witt (pxf5695@autuni.ac.nz)
* Created: 15/09/2017 11:17:34 PM
*
* Project Repository: https://github.com/wittsend/aut-swarm-robotics
*
* Retrieves and handles messages from the Xbee network
*
* More Info:
* Atmel SAM 4N Processor Datasheet:http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
* Relevant reference materials or datasheets if applicable
*
* Functions:
* void commGetNew()
* void commInterpretSwarmMessage(struct MessageInfo message, RobotGlobalStructure *sys)
*
*/

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "../robot_setup.h"

#include "../Interfaces/xbee_driver.h"
#include "../Interfaces/twimux_interface.h"

#include "navigation_functions.h"		//Updating robot position
#include "motion_functions.h"
#include "comm_functions.h"


#include <stdlib.h>
#include <stdio.h>						//sprintf
#include <string.h>						//strcpy


//////////////[Functions]///////////////////////////////////////////////////////////////////////////
/*
* Function:
* void commGetNew(struct FrameInfo *frame, struct MessageInfo *message)
*
* Checks for new communications and handles the interpretation of them
*
* Inputs:
* pointer to frame_info struct and pointer to message_info struct
*
* Returns:
* none
*
* Implementation:
* First, checks if communication polling is enabled, and if the polling interval has elapsed.
* If so, checks if the XBee frame buffer is full indicating new data is ready to be read
* If so, interpret the new XBee API frame and use to fill the message buffer
* Then is the message buffer is full interpret the swarm message
*
*/
/*
void commGetNew(RobotGlobalStructure *sys)
{
	static uint32_t nextPollTime = 0;
	struct FrameInfo commFrame;
	
	//If polling is enabled and the poll interval has elapsed
	if(sys->comms.pollEnabled && sys->timeStamp >= nextPollTime)
	{
		//Set the time at which comms will next be polled
		nextPollTime = sys->timeStamp + sys->comms.pollInterval;
		if(!xbeeFrameBufferInfoGetFull(&commFrame))			//Check for a received XBee Message
		{
			xbeeInterpretAPIFrame(commFrame);				//Interpret the received XBee Message
			if(!xbeeMessageBufferInfoGetFull(&sys->comms.messageData))//if message from the swarm
				commInterpretSwarmMessage(sys);				//Interpret the message
		}
		
		if(sys->comms.twi2SlavePollEnabled)					//If polling TWI2 Slave reqs is enabled
		{
			commTwi2SlaveRequest(sys);
		}
	}
}
*/

void commGetNew(RobotGlobalStructure *sys)
{
	static uint32_t nextPollTime = 0;
	
	// If checking of slave requests on twi2 (from top mounted LCD) is enabled and the poll interval has elapsed
	if(sys->comms.twi2SlavePollEnabled && sys->timeStamp >= nextPollTime)
	{
		nextPollTime = sys->timeStamp + sys->comms.twi2SlavePollInterval;
		commTwi2SlaveRequest(sys);
	}
	
	// If there is new xbee Data
	if(sys->comms.xbeeNewData)
	{
		commInterpretSwarmMessage(sys);	// interpret the message
		sys->comms.xbeeNewData = false;
	}		
}



/*
* Function:
* void commInterpretSwarmMessage(struct MessageInfo message, RobotGlobalStructure *sys)
*
* Interprets and acts on a received swarm messages
*
* Inputs:
* RobotGlobalStructure *sys:
*   Pointer to the global system data structure
*
* Returns:
* none
*
* Implementation:
* TODO: Adam: implementation description
*
* Improvements:
* Improvements: is it worth defining all these codes, im thinking no << There should be NO magic
* numbers.
*
*/
void commInterpretSwarmMessage(RobotGlobalStructure *sys)
{
	switch(sys->comms.xbeeMessageType & 0xF0)	//Look at upper nibble only
	{
		//PositionGroup commands
		case RX_UPDATE_POSITION:
			switch(sys->comms.xbeeMessageType & 0x0F)
			{
				//X, Y position from PC
				case 0x00:
					nfApplyPositionUpdateFromPC(sys);
					break;
			}
			break;
		
		//Test commands
		case RX_TEST_MODE:
			if(sys->states.mainf != M_TEST)
			{
				sys->states.mainfPrev = sys->states.mainf;
			}
			sys->states.mainf = M_TEST;
			break;

		//Manual control
		case RX_MANUAL_MODE:
			switch(sys->comms.xbeeMessageType & 0x0F)	//Look at lower nibble only
			{
				case RX_M_STOP:
				case RX_M_MOVE:
				case RX_M_ROTATE_CW:
				case RX_M_ROTATE_CCW:
					if(sys->states.mainf != M_MANUAL)
					{
						sys->states.mainfPrev = sys->states.mainf;
					}
					sys->states.mainf = M_MANUAL;
					break;
					

				case RX_M_RANDOM:
					//move robot randomly
					sys->states.mainfPrev = sys->states.mainf;
					sys->states.mainf = M_RANDOM;
					break;
					
				case 0x05:
					//0xD5 is reserved 
					break;
					
				case RX_M_RELEASE_DOCK:
					if(sys->states.mainf == M_CHARGING)
					{
						sys->states.chargeCycle = CCS_DISMOUNT;
					}
					break; 
					
				case RX_M_DOCK:
					//sys->states.dockingLight = DS_FINISHED;
					//sys->states.mainf = M_DOCKING_OLD;
					sys->states.dockingCam = DCS_START;
					sys->states.mainfPrev = sys->states.mainf;
					sys->states.mainf = M_DOCKING;
					break;

				case 0x08:
					//0xD8 is reserved 
					break;

				case RX_M_OBSTACLE_AVOIDANCE:
					sys->states.mainfPrev = sys->states.mainf;
					sys->states.mainf = M_OBSTACLE_AVOIDANCE_DEMO;
					break;

				case RX_M_LIGHT_FOLLOW:
					sys->states.mainfPrev = sys->states.mainf;
					sys->states.mainf = M_LIGHT_FOLLOW;
					break;

				case RX_M_LINE_FOLLOW:
					sys->states.mainfPrev = sys->states.mainf;
					sys->states.mainf = M_LINE_FOLLOW;
					break;
					
				case RX_M_ROTATE_TO_HEADING:					
					sys->pos.targetHeading = (float)((int16_t)((sys->comms.xbeeMessageData[0]<<8)|(sys->comms.xbeeMessageData[1])));
					sys->states.mainfPrev = sys->states.mainf;
					sys->states.mainf = M_ROTATE_TO_FACING;
					break;
					
				case RX_M_MOVE_TO_POSITION:
					sys->pos.targetX = (sys->comms.xbeeMessageData[2]<<8)|sys->comms.xbeeMessageData[3];
					sys->pos.targetY = (sys->comms.xbeeMessageData[0]<<8)|sys->comms.xbeeMessageData[1];
					sys->states.mainfPrev = sys->states.mainf;
					sys->states.mainf = M_MOVE_TO_POSITION;
					break;

				case 0x0E:
					//0xDE is reserved
					break;

				case 0x0F:
					//0xDF is reserved
					break;
			}
			break;
	}
}

/*
* Function:
* void commSendDebugString(char string[])
*
* Sends a debug string back to the GUI
*
* Inputs:
* char string[]:
*   string to send
*
* Returns:
* none
*
* Implementation:
* Copies the string to a buffer and then uses the correct xbee function call to send the string
*
*/
void commSendDebugString(char string[])
{

	uint8_t string_length = strlen(string);
	char message_data[string_length + 3];

	message_data[0] = 0x00;				//Command letting PC know of debug message
	message_data[1] = 0x00;				//string only
	message_data[2]	= string_length;	//the length of the string
	strcpy(message_data + 3, string);
	xbeeSendAPITransmitRequest(COORDINATOR_64, UNKNOWN_16, message_data, string_length + 3);  //Send the Message
	
}

/*
* Function:
* void commSendDebugFloat(char variableName[], float variable)
*
* Sends a debug string back to the GUI
*
* Inputs:
* char string[]:
*   string to send
* float variable
*	variable to send
*
* Returns:
* none
*
* Implementation:
* Copies the string and variable to a buffer and then uses the correct xbee function call to send the string
*
*/
void commSendDebugFloat(char variableName[], float variable)
{

	uint8_t string_length = strlen(variableName);
	char message_data[string_length + 3 + sizeof(variable)];

	message_data[0] = 0x00;													//Command letting PC know of debug message
	message_data[1] = 0x01;													//string and float
	message_data[2]	= string_length;										//the length of the string
	strcpy(message_data + 3, variableName);									//copy the string to the message data
	memcpy(message_data + string_length + 3, &variable, sizeof(variable));	//copy the variable to the message data

	xbeeSendAPITransmitRequest(COORDINATOR_64, UNKNOWN_16, message_data, sizeof(message_data));  //Send the Message

}

/*
* Function:
* void commSendDebugFloatWithTimestamp(char variableName[], float variable)
*
* Sends a debug string back to the GUI
*
* Inputs:
* char string[]:
*   string to send
* float variable
*	variable to send
*
* Returns:
* none
*
* Implementation:
* Copies the string and variable to a buffer and then uses the correct xbee function call to send the string
*
*/
void commSendDebugFloatWithTimestamp(char variableName[], float variable)
{

	uint8_t string_length = strlen(variableName);
	float timestamp = (float) sys.timeStamp / 1000.0;
	char message_data[string_length + 7 + sizeof(variable)];

	message_data[0] = 0x00;													//Command letting PC know of debug message
	message_data[1] = 0x02;													//string, timestamp and float
	message_data[2]	= string_length;										//the length of the string
	strcpy(message_data + 3, variableName);									//copy the string to the message data
	memcpy(message_data + string_length + 3, &timestamp, 4);				//copy the timestamp to the message data
	memcpy(message_data + string_length + 7, &variable, sizeof(variable));	//copy the variable to the message data

	xbeeSendAPITransmitRequest(COORDINATOR_64, UNKNOWN_16, message_data, sizeof(message_data));  //Send the Message

}

/*
* Function:
* char commTwi2SlaveRequest()
*
* Checks for a request from a master on TWI2 and acts on it (for the LCD interface)
*
* Inputs:
* none
*
* Returns:
* 0 on success
*
* Implementation:
* [[[WIP]]]
*
* Improvements:
* [Ideas for improvements that are yet to be made](optional)
*
*/
char commTwi2SlaveRequest(RobotGlobalStructure *sys)
{
	uint8_t outputBuffer = 0;
	if(sys->flags.twi2NewData && twi2SlaveAccess)
	{
		if(twi2SlaveReadMode)
		{
			sys->flags.twi2NewData = 0;

			switch(sys->comms.twi2ReceivedDataByte)
			{
				case COMM_TWI2_ROBOT_NAME:				//Commands go here
					outputBuffer = 0;				
					break;
								
				case COMM_TWI2_BATTERY_LVL:				//Commands go here
					outputBuffer = sys->power.batteryPercentage;				
					break;
				
				case COMM_TWI2_HEADING:				//Commands go here
					outputBuffer = (uint8_t)((sys->pos.relHeading + 180)/2);
					//outputBuffer = (((uint16_t)sys->pos.heading) >> 2);
					break;
				
				case COMM_TWI2_OPTX:				//Commands go here
					outputBuffer = (uint8_t)((sys->pos.Optical.x & 0xFF0000) >> 16);
					break;
				
				case COMM_TWI2_OPTY:				//Commands go here
					outputBuffer = (uint8_t)((sys->pos.Optical.y & 0xFF0000) >> 16);
					break;
				
				case COMM_TWI2_FACING:				//Commands go here
					outputBuffer = (uint8_t)((sys->pos.facing + 180)/2);				
					break;
				
				case COMM_TWI2_COLOUR:
					outputBuffer = (uint8_t)((sys->sensors.colour.left.hue)/2);
					break;
				
				default:
					outputBuffer = 0;
					break;

			}
			twi2Send(outputBuffer);
			while(!twi2TxReady);			//Wait for flag
			while(!twi2TxComplete);

		}
	}
	return 0;
} 


//TODO: Commenting. send battery and task to PC
void commPCStatusUpdate(RobotGlobalStructure *sys)
{
	//When to next send update
	static uint32_t updateNextTime = 0;

	if((sys->timeStamp > updateNextTime) && sys->comms.pcUpdateEnable)
	{
		updateNextTime = sys->timeStamp + sys->comms.pcUpdateInterval;
		sys->comms.transmitData.Data[0] = 0xA1; //Command letting PC know of update
		sys->comms.transmitData.Data[1] = sys->states.mainf; //Robot State
		sys->comms.transmitData.Data[2] = sys->power.batteryVoltage >> 8; //Upper byte
		sys->comms.transmitData.Data[3] = sys->power.batteryVoltage & 0xFF; //Lower byte
		sys->comms.transmitData.DataSize = 4;
		xbeeSendAPITransmitRequest(COORDINATOR_64,UNKNOWN_16, sys->comms.transmitData.Data, 
									sys->comms.transmitData.DataSize);  //Send the Message
		
		//DEBUG MESSAGE (please don't delete - Matt):
		/*
		//char stringBuffer[49];
		sys->comms.transmitData.Data[0] = 0x00;
		dtoa(stringBuffer, (double)sys->pos.Optical.convFactor);
		
		strcpy(sys->comms.transmitData.Data + 1, stringBuffer);
		
		sys->comms.transmitData.DataSize = strlen(stringBuffer) + 2;
		xbeeSendAPITransmitRequest(COORDINATOR_64,UNKNOWN_16, sys->comms.transmitData.Data,
									sys->comms.transmitData.DataSize);  //Send the Message	
		*/
	}
}