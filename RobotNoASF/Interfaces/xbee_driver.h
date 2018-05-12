/*
* xbee_driver.h
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
*
*/

#ifndef XBEE_DRIVER_H_
#define XBEE_DRIVER_H_

//////////////[Defines]/////////////////////////////////////////////////////////////////////////////
// XBee flow control bytes
#define FRAME_DELIMITER							0x7E
#define ESCAPE_BYTE								0x7D
#define XON										0x11
#define XOFF									0x13

// XBee API Frames
#define AT_COMMAND								0x08
#define AT_COMMAND_QUEUE						0x09
#define ZIGBEE_TRANSMIT_REQUEST					0x10
#define EXPLICIT_ADDRESSING_ZIGBEE_COMMAND_FRAME 0x11
#define REMOTE_COMMAND_REQUEST					0x17
#define CREATE_SOURCE_ROUTE						0x21
#define AT_COMMAND_RESPONSE						0x88
#define MODEM_STATUS							0x8A
#define ZIGBEE_TRANSMIT_STATUS					0x8B
#define ZIGBEE_RECEIVE_PACKET					0x90
#define ZIGBEE_EXPLICIT_RX_INDICATOR			0x91
#define ZIGBEE_IO_DATA_SAMPLE_RX_INDICATOR		0x92
#define XBEE_SENSOR_READ_INDICATOR				0x94
#define NODE_IDENTIFICATION_INDICATOR			0x95
#define REMOTE_COMMAND_RESPONSE					0x97
#define EXTENDED_MODEM_STATUS					0x98
#define OTA_FIRMWARE_UPDATE_STATUS				0xA0
#define ROUTE_RECORD_INDICATOR					0xA1
#define MANY_TO_ONE_ROUTE_REQUEST_INDICATOR		0xA3

//64-bit robot addresses
#define COORDINATOR_64							0x0000000000000000
#define BROADCAST_64							0x000000000000FFFF

//16-bit robot addresses
#define ALL_ROUTERS_16							0xFFFC
#define ALL_NON_SLEEPING_DEVICES_16				0xFFFD
#define UNKNOWN_16								0xFFFE
#define ALL_DEVICES_16							0xFFFF

//TODO: Message codes
//System Messages 0x0*
//Navigation system Messages 0x1*
#define NAV_IMU_QW								0x10
#define NAV_IMU_QX								0x11
#define NAV_IMU_QY								0x12
#define NAV_IMU_QZ								0x13
#define NAV_IMU_DYAW							0x14
#define NAV_OPT_DX								0x15
#define NAV_OPT_DY								0x16
#define NAV_OPT_VEL								0x17
#define NAV_OPT_HDG								0x18
//Robot Control Messages 
//Test Messages

//////////////[Type Definitions]////////////////////////////////////////////////////////////////////

//////////////[Global variables]////////////////////////////////////////////////////////////////////

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
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
*/
void xbeeInit(void);

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
*/
void xbeeSendAPITransmitRequest(uint64_t destination_64, uint16_t destination_16, char *data, uint8_t  bytes);

#endif /* XBEE_DRIVER_H_ */
