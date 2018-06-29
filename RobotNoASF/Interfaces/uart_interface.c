/*
* uart_interface.c
*
* Author : Mansel Jeffares/Matthew Witt (pxf5695@autuni.ac.nz)
* Created: 27/08/2017 4:15:14 PM
*
* Project Repository: https://github.xbee/wittsend/aut-swarm-robotics
*
* Functions for initialising and accessing the UART interface. Also contains interrupt handlers
* for the UART interface.
*
* More Info:
* Atmel SAM 4N Processor Datasheet:http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
* Relevant reference materials or datasheets if applicable
*
* Functions:
* void uart3Init(void)
* uint8_t uart3Write(uint8_t data)
* void UART3_Handler(void)
*
*/

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "../robot_setup.h"
#include "uart_interface.h"
#include "xbee_driver.h"
#include <string.h>

#include "../functions/comm_functions.h"


//////////////[Global Variables]////////////////////////////////////////////////////////////////////
extern RobotGlobalStructure sys;	//For UART interrupt

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
/*
* Function:
* void uart3Init(void)
*
* Initialises UART3 peripheral, ready for use. Used by the Xbee on the robot.
*
* Inputs:
* none
*
* Returns:
* none
*
* Implementation:
* - Peripheral clock access is supplied to UART3
* - UART3 is given control of PIOB PB10 (RX) and PB11 (TX)
* - No parity mode enabled
* - 9600bps
* - Reset the UART3 control register
* - Enable UART3
* - Enable the RX ready interrupt
*
*/
void uart3Init(void)
{
	REG_PMC_PCER0
	|=	(1<<ID_UART3);				//Enable clock access to UART3
	REG_PIOB_PDR
	|=	PIO_PB10					//Enable peripheral control of PB10 (URXD3)
	|	PIO_PB11;					//And PB11 (UTXD3) both connected to peripheral B
	REG_UART3_MR
	|=	UART_MR_CHMODE_NORMAL 
	|	UART_MR_PAR_NO;				//set as no parity, normal mode
	REG_UART3_BRGR
	//=	651;						//Set Baud rate for 9600 from a 100MHZ clock
	=	27;							//Baud 230400
	REG_UART3_CR
	|=	UART_CR_RSTRX
	|	UART_CR_RSTTX
	|	UART_CR_RSTSTA;				//Reset receiver, transmitter and status bits
	REG_UART3_CR
	|=	UART_CR_RXEN
	|	UART_CR_TXEN;				//Enable receiver and transmitter
	REG_UART3_IER
	|=	UART_IER_RXRDY;				//ENABLE UART RXRDY interrupt
	
	NVIC_EnableIRQ(ID_UART3);		//ENABLE the UART3 system interrupts	
}

/*
* Function:
* uint8_t uart3Write(uint8_t data)
*
* Allows a byte to be written to UART3
*
* Inputs:
* uint8_t data:
*   The byte to be written to the UART
*
* Returns:
* 0 on success, otherwise non zero if failed
*
* Implementation:
* Calls waitForFlag() to make sure that the UART3 TXRDY flag is set, meaning data can be placed in
* the transmit holding register. If waitForFlag() returns a 0, then place data in the transmit
* holding register. Return the value returned from waitForFlag().
*
*/
uint8_t uart3Write(uint8_t data)
{
	//Wait till TXRDY
	uint8_t returnVal = waitForFlag((uint32_t*)&REG_UART3_SR, UART_SR_TXRDY, UART_TXRDY_TIMEOUT); 
	if(!returnVal)
		REG_UART3_THR = data;				//place data in TX register
	return returnVal;
}

/*
* Function:
* void UART3_Handler(void)
*
* Interrupt handler for UART3
*
* Inputs:
* none
*
* Returns:
* none
*
* Implementation:
*	Checks to see that we have data ready to be received
*	Stores the incoming data into a temporary variable
*	Checks if we receive a start delimiter out of sequence
*	Checks if we receive a escape byte
*	If the data needs to be escaped we calculate the correct byte
*	Then we move through a receive state machine for the xbee frame with states for: start, length, data, checksum
*	
*/
void UART3_Handler(void)
{
	//Receive States
	enum {START, LENGTH_MSB, LENGTH_LSB, FRAME_TYPE, DATA, CHECKSUM};	//const

	uint8_t rxData;	//temporary variable to store received byte
	
	static uint8_t receiveState = START;
	static bool escape = false;		// Flag for escaping received bytes
	static uint16_t length;			// Length as reported by XBee Frame
	static uint8_t index;			// Number of bytes received that count towards length of XBee Frame
	static int check;				// Checksum calculation
	static uint8_t frame_type;		// The XBEE message type
	static uint8_t message_type;	// The swarm message type
	static uint8_t rxBuffer [100];	// Array to buffer incoming messages
	

	if(REG_UART3_IMR & UART_IMR_RXRDY)	// If we receive data
	{
		rxData = REG_UART3_RHR;	// Store the incoming data in a temporary variable

		if(rxData == FRAME_DELIMITER && receiveState != START )	// If we receive a start byte out of sequence
		{
			receiveState = START;								// Reset back to the start state
		}
		else if(rxData == ESCAPE_BYTE)							// If the next byte needs to be escaped
		{
			escape = true;										// Set the flag
		}
		else if(escape)											// If the current byte needs to be escaped
		{
			rxData ^= 0x20;	// Reverse the escape procedure
			escape = false;	// Reset the flag
		}

		if(escape == false)	//we only go through the receive states if the data has been escaped
		{
			switch(receiveState)
			{
				case START:
					if(rxData == FRAME_DELIMITER)
					{
						//reset our book-keeping variables and updates the receive state
						length = 0;
						index = 0;
						check = 0;
						receiveState = LENGTH_MSB;
					}
					break;

				case LENGTH_MSB:
					//Calculates the length using the first length byte and updates the receive 
					//state
					length = rxData * 256;
					receiveState = LENGTH_LSB;
					break;

				case LENGTH_LSB:
					//Calculates the length using the second length byte and updates the receive 
					//state
					length =+ rxData;
					receiveState = FRAME_TYPE;
					break;

				case FRAME_TYPE:
					frame_type = rxData;			//Receives and stores the Frame type
					check += rxData;				//Calculates the checksum over the received byte
					index++;					//Updates the number of received bytes that count
												//towards the XBee frame length
					
					//if we receive a message we care about we go and get the rest of the data 
					if(frame_type == ZIGBEE_RECEIVE_PACKET)
					{
						receiveState = DATA;		//Updates the receive state
					}
					else
					{
						receiveState = START;		//Updates the receive state
					}
					
					break;

				case DATA:
					//xbeeFrameBufferPut(temp);	//Stores the Received data into the FrameBuffer
					check += rxData;				//Calculates the checksum over the received byte
					index++;					//Updates the number of received bytes that count 
												//towards the XBee frame length

					// we don't store the 64-bit source address (8bytes)
					// we don't store the 16-bit source address (2bytes)
					// we don't store the receive options (1byte)
					// the frame type has already been used (1byte)
					if(index == 13)	
					{
						message_type = rxData;
					}
					else if(index > 13)
					{
						rxBuffer[index - 14] = rxData;
						//xbeeFrameBufferPut(rxData);
					}
					
					// Checks if we have received all the data and if we have updates the receive state
					if(index == length)		
					{
						receiveState = CHECKSUM;
					}
				
					break;
				
				case CHECKSUM:
					check += rxData;			//Calculates the checksum over the received byte
					check &= 0xFF;				//Final Step of checksum calculation for XBee Frame
					if(check == 0xFF)			//Verifies the calculated checksum value
					{
						//If there is still data waiting to be interpreted we have missed a message
						if(sys.comms.xbeeNewData)
						{
							sys.comms.xbeeMissedMessages++;
							//commSendDebugFloatWithTimestamp("Xbee Missed Messages", (float) (sys.comms.xbeeMissedMessages), &sys);
						}
						else
						{
							sys.comms.xbeeNewData = true;
							sys.comms.xbeeMessageType = message_type;
							memcpy(sys.comms.xbeeMessageData, rxBuffer, length - 13);
						}
					}
					receiveState = START;		//Resets receive state 
					break;
			}
		}
	}
}
