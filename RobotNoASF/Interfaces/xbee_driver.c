/*
* xbee_driver.c
*
* Author : Mansel Jeffares/Matthew Witt (pxf5695@autuni.ac.nz)
* Created: 11 July 2017
*
* Project Repository: https://github.com/wittsend/aut-swarm-robotics
*
* Communication to computer GUI using wireless XBee Module in API Mode
*
* More Info:
* Atmel SAM 4N Processor Datasheet:http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
* Relevant reference materials or datasheets if applicable
*
* Functions:
* void xbeeInit(void)
* void xbeeSendAPITransmitRequest(uint64_t destination_64, uint16_t destination_16,
*                                 uint8_t *data, uint8_t  bytes)
* static void xbeeSendAPIFrame(uint8_t * frame_data, int len)
*
*/

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "../robot_setup.h"

#include "xbee_driver.h"
#include "uart_interface.h"

#include "../Functions/motion_functions.h"	//For interpretswarmmsg.. will be moved

#include <string.h>
#include <stdio.h>

//////////////[Global Variables]////////////////////////////////////////////////////////////////////

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
//Private function prototypes. See below for function descriptions
static void xbeeSendAPIFrame(uint8_t * frame_data, int len);

/*
* Function:
* void xbeeInit(void)
*
* Sets up UART3 and the required buffers
*
* Inputs:
* none
*
* Returns:
* none
*
* Implementation:
*	Calls UART 3 initialisation function to correctly setup the UART for Xbee communications
*	Initializes our various buffers using the appropriate functions
*
*/
void xbeeInit(void)
{
	//Sets up the uart3 for communication to the Xbee
	uart3Init();
}

/*
* Function:
* void xbeeSendAPITransmitRequest(uint64_t destination_64, uint16_t destination_16, 
*                                 uint8_t *data, uint8_t  bytes)
*
* Forms and sends XBee Transmit Request Frame
*
* Inputs:
* uint64_t destination_64:
*   MSB first, the Zigbee 64bit address of the destination device. Reserved addresses include: the coordinator = 0x0000000000000000, broadcast = 0x000000000000FFFF
* uint16_t destination_16:
*   MSB, first, the Zigbee 16bit address of the destination device.Reserved addresses include: unknown or broadcast = 0xFFFE,  broadcast to routers = 0xFFFC, broadcast to all non-sleeping devices = 0xFFFD, broadcast to all devices including sleeping end devices = 0xFFFE.
* uint8_t *data:
*   pointer to an array of data to be sent to the destination device
* uint8_t  bytes:
*   The number of bytes to send
*
* Returns:
* none
*
* Implementation:
* Copies data into a single array for the correct format required by the Xbee protocol
*
*/
void xbeeSendAPITransmitRequest(uint64_t destination_64, uint16_t destination_16,
								char *data, uint8_t  bytes)
{
	//array to store message
	uint8_t frame_data[bytes + 14];

	frame_data[0] = ZIGBEE_TRANSMIT_REQUEST;	//Frame type
	frame_data[1] = 150;						//frame ID (assigned arbitrary at the moment)

	//Destination 64-Bit Address
	frame_data[2] = (destination_64 & (0xFF00000000000000)) >> 56;
	frame_data[3] = (destination_64 & (0xFF000000000000)) >> 48;
	frame_data[4] = (destination_64 & (0xFF0000000000)) >> 40;
	frame_data[5] = (destination_64 & (0xFF00000000)) >> 32;
	frame_data[6] = (destination_64 & (0xFF000000)) >> 24;
	frame_data[7] = (destination_64 & (0xFF0000)) >> 16;
	frame_data[8] = (destination_64 & (0xFF00)) >> 8;
	frame_data[9] = destination_64 & (0xFF);

	//Destination 16-Bit Address
	frame_data[10] = (destination_16 & (0xFF00)) >> 8;
	frame_data[11] = destination_16 & (0xFF);

	//Broadcast radius
	frame_data[12] = 0x00;  

	//Options
	frame_data[13] = 0x00;  

	//Copies message data to frame array
	memcpy(frame_data + 14, data, bytes);

	//Sends the message
	xbeeSendAPIFrame(frame_data,bytes+14);
}

/*
* Function:
* static void xbeeSendAPIFrame(uint8_t * frame_data, int len)
*
* Sends an XBee API Frame
*
* Inputs:
* uint8_t *frame_data:
*   The data of the xbee frame to be sent
* int len:
*   The length of the frame data
*
* Returns:
* none
*
* Implementation:
* Adds the xbee header: frame delimiter and length, frame data and calculates the checksum and places it all in a single array
* Sends the entire frame to the xbee over the uart escaping the characters as required by xbee API mode 2
*
*/
static void xbeeSendAPIFrame(uint8_t *frame_data, int len)
{
	uint16_t length = len;		//length of API Frame Data
	uint8_t data[length + 4];	//Array to store the full Frame
	uint8_t checksum = 0;		//Variable to store checksum calculation
	uint8_t data_length;

	//Forms XBee Frame Header with start delimiter and length
	data[0] = FRAME_DELIMITER;
	data[1] = (uint8_t) (length >> 7) & 0xFF;
	data[2] = length & 0xFF;

	//Copies frame data into the full frame array
	memcpy(data + 3, frame_data, length);
	
	//Calculates the checksum over the frame data array
	for(int i = 0; i < length; i++)
	{
		checksum += frame_data[i];
	}
	
	data[length+3] = 0xFF - checksum;	//Completes final step in checksum calculation and copies it to the full frame array
	data_length = sizeof(data);			//Gets the length of full message
	uart3Write(data[0]);				//Writes the Frame Delimiter as this shouldn't be escaped

	//Sends the message out the UART, escaping characters on the fly as needed
	for(int i = 1; i <data_length; i++)
	{
		//Checks for bytes that need to be escaped
		if(data[i] == 0x7E || data[i] == 0x7D || data[i] == 0x11 || data[i] == 0x13)
		{
			uart3Write(ESCAPE_BYTE);	//Writes the escape byte
			uart3Write(data[i]^0x20);	//Writes the escaped byte
		}
		else
		{
			uart3Write(data[i]);	//Writes the byte
		}
	}
}
