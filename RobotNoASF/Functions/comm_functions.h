/*
* comm_functions.h
*
* Author : Adam Parlane/Matthew Witt (pxf5695@autuni.ac.nz)
* Created: 15/09/2017 11:17:47 PM
*
* Project Repository: https://github.com/wittsend/aut-swarm-robotics
*
* Header file for comm_functions.c
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

#ifndef COMM_FUNCTIONS_H_
#define COMM_FUNCTIONS_H_

//////////////[Defines]/////////////////////////////////////////////////////////////////////////////
//TWI slave internal register addresses

#define COMM_TWI2_ROBOT_NAME	0x11
#define COMM_TWI2_BATTERY_LVL	0x21
#define COMM_TWI2_HEADING		0x31
#define COMM_TWI2_OPTX			0x32
#define COMM_TWI2_OPTY			0x33
#define COMM_TWI2_FACING		0x34
#define COMM_TWI2_COLOUR			0x41

//defines for received modes
#define RX_UPDATE_POSITION		0xA0
#define RX_TEST_MODE			0xE0
#define RX_MANUAL_MODE			0xD0

//defines for received manual command states
#define RX_M_STOP					0x00
#define RX_M_MOVE					0x01
#define RX_M_ROTATE_CW				0x02
#define RX_M_ROTATE_CCW				0x03
#define RX_M_RANDOM					0x04
//0x05 is reserved 
#define RX_M_RELEASE_DOCK			0x06
#define RX_M_DOCK					0x07
//0x08 is reserved 
#define RX_M_OBSTACLE_AVOIDANCE		0x09
#define RX_M_LIGHT_FOLLOW			0x0A
#define RX_M_LINE_FOLLOW			0x0B
#define RX_M_ROTATE_TO_HEADING		0x0C
#define RX_M_MOVE_TO_POSITION		0x0D
//0x0E is reserved 
//0x0F is reserved 


//////////////[Functions]///////////////////////////////////////////////////////////////////////////
/*
* Function:
* void commGetNew(RobotGlobalStructure *sys)
*
* Checks for new communications and handlles the interpretation of them
*
* Inputs:
* pointer to frame_info struct and pointer to message_info struct
*
* Returns:
* none
*
*/
void commGetNew(RobotGlobalStructure *sys);

/*
* Function:
* void commInterpretSwarmMessage(RobotGlobalStructure *sys)
*
* Interprets and acts on a received swarm messages
*
* Inputs:
* struct MessageInfo message:
*   TODO: Adam input description
*
* Returns:
* none
*
*/
void commInterpretSwarmMessage(RobotGlobalStructure *sys);

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
void commSendDebugString(char string[]);

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
void commSendDebugFloat(char variableName[], float variable);

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
void commSendDebugFloatWithTimestamp(char variableName[], float variable);

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
*/
char commTwi2SlaveRequest(RobotGlobalStructure *sys);

//send battery and task to PC
void commPCStatusUpdate(RobotGlobalStructure *sys);

#endif /* COMM_FUNCTIONS_H_ */