/*
* twimux_interface.c
*
* Author : Esmond Mather and Matthew Witt
* Created: 11/07/2017 10:41:59 AM
*
* Project Repository:https://github.com/wittsend/aut-swarm-robotics
*
* Functions for reading and writing I2C devices on TWI0 bus. Functions for controlling I2C
* multiplexer on TWI0.
*
* More Info:
* Atmel SAM 4N Processor Datasheet:http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
* Farnell I2C Mux Datasheet:http://www.farnell.com/datasheets/1815500.pdf
*
* Functions:
* void twi0Init(void)
* void twi2Init(void)
* void twi0MuxReset(void)
* uint8_t twi0MuxSwitch(uint8_t channel)
* uint8_t twi0ReadMuxChannel(void)
* uint8_t twi0SetCamRegister(uint8_t regAddr)
* uint8_t twi0ReadCameraRegister(void)
* char twi0Write(unsigned char slave_addr, unsigned char reg_addr,
*						unsigned char length, unsigned char const *data)
* char twi2Write(unsigned char slave_addr, unsigned char reg_addr,
*						unsigned char length, unsigned char const *data)
* char twi0Read(unsigned char slave_addr, unsigned char reg_addr,
*						unsigned char length,	unsigned char *data)
* char twi2Read(unsigned char slave_addr, unsigned char reg_addr,
*						unsigned char length,	unsigned char *data)
* uint8_t twi0LogEvent(TwiEvent event)
*
*/

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "timer_interface.h"			//mux reset delay
#include "pio_interface.h"				//Access to LEDs for debugging
#include "twimux_interface.h"

//////////////[Defines]/////////////////////////////////////////////////////////////////////////////
//TWI Multiplexer Reset Pin definition (Experimental feature on brown bot)
#define TWIMUX_RESET_PORT		PIOC
#define TWIMUX_RESET_PIN		PIO_PC26

#define TWIBB_LOW_TIME	5		//Clock low time (us)
#define TWIBB_HIGH_TIME	5		//Clock high time (us)
#define TWIBB_SR_DELAY	5		//Repeat start delay (us)


#define twi0ClkLow		(PIOA->PIO_CODR |= PIO_PA4)
#define twi0ClkHigh		(PIOA->PIO_SODR |= PIO_PA4)
#define twi0ClkTog		{if(PIOA->PIO_ODSR&PIO_PA4) twi0ClkLow; else twi0ClkHigh;}
#define twi0DataLow		(PIOA->PIO_CODR |= PIO_PA3)
#define twi0DataHigh	(PIOA->PIO_SODR |= PIO_PA3)
#define twi0DataTog		{if(PIOA->PIO_ODSR&PIO_PA3) twi0DataLow; else twi0DataHigh;}
#define twi0DataGet		((PIOA->PIO_PDSR&PIO_PA3)>>3)
#define twi0ClkGet		((PIOA->PIO_PDSR&PIO_PA4)>>4)
	
#define	twi2ClkOn 		(PIOB->PIO_SODR |= PIO_PB1)
#define	twi2ClkOff 		(PIOB->PIO_CODR |= PIO_PB1)
#define twi2ClkTog		{if(PIOB->PIO_ODSR&PIO_PB1) twi2ClkOff; else twi2ClkOn;}


	
//TWI Multiplexer reset macros
#define twiMuxSet				TWIMUX_RESET_PORT->PIO_SODR |= TWIMUX_RESET_PIN
#define twiMuxReset				TWIMUX_RESET_PORT->PIO_CODR |= TWIMUX_RESET_PIN



//////////////[Global Variables]////////////////////////////////////////////////////////////////////
extern RobotGlobalStructure sys;		//Gives TWI2 interrupt handler access
TwiEvent twi0Log[TWI_LOG_NUM_ENTRIES];	//TWI0 event log

//////////////[Private Functions]///////////////////////////////////////////////////////////////////
void twi0bbStart(void)
{
	twi0DataLow;
	delay_us(5);
	twi0ClkLow;
	delay_us(TWIBB_LOW_TIME);
}

void twi0bbRepeatStart(void)
{
	twi0ClkLow;
	twi0DataHigh;
	delay_us(TWIBB_LOW_TIME);
	twi0ClkHigh;
	delay_us(TWIBB_HIGH_TIME/2);
	twi0DataLow;
	delay_us(TWIBB_HIGH_TIME/2);
	twi0ClkLow;
	delay_us(TWIBB_LOW_TIME);
}

void twi0bbStop(void)
{
	delay_us(TWIBB_LOW_TIME);
	twi0ClkHigh;
	delay_us(5);
	twi0DataHigh;
	delay_us(TWIBB_HIGH_TIME);
}

uint8_t twi0bbSendByte(uint8_t data)
{
	uint8_t ack = 0;
	//Shift out the bits of the byte
	for(int8_t b = 7; b>=0; b--)
	{
		twi0ClkLow;
		if(data&(1<<b))
		twi0DataHigh;
		else
		twi0DataLow;
		delay_us(TWIBB_LOW_TIME);
		twi0ClkHigh;
		delay_us(TWIBB_HIGH_TIME);
	}
	
	//Look for acknowledge from slave
	twi0ClkLow;					//Clock low
	twi0DataHigh;				//Release data line
	delay_us(TWIBB_LOW_TIME);
	twi0ClkHigh;
	delay_us(1);
	ack = !twi0DataGet;			//Read status of data line
	delay_us(TWIBB_HIGH_TIME-1);
	twi0ClkLow;

	return ack;
}

uint8_t twi0bbReadByte(uint8_t *data)
{
	uint8_t ack = 0;
	*data = 0x0;
	twi0DataHigh;		//Release data line
	for(int8_t b=7; b>=0; b--)
	{
		twi0ClkLow;
		delay_us(TWIBB_LOW_TIME);
		twi0ClkHigh;
		*data |= (twi0DataGet<<b);
		delay_us(TWIBB_HIGH_TIME);
	}
	
	//Get acknowledge from slave
	twi0ClkLow;
	delay_us(TWIBB_LOW_TIME);
	twi0ClkHigh;
	delay_us(1);
	ack = !twi0DataGet;
	delay_us(TWIBB_HIGH_TIME-1);
	twi0ClkLow;
	return ack;
}
//////////////[Public Functions]////////////////////////////////////////////////////////////////////
/*
* Function:
* void twi0Init(void);
*
* Initialises TWI0. Master clock should be setup first.
*
* Inputs:
* none
*
* Returns:
* none
*
* Implementation:
* - Supply master clock to TWI0 peripheral
* - Set PA3(Data) and PA4(Clock) to peripheral A mode (TWI)
* - Set the high and low periods of the TWI clock signal using formula from datasheet
*	NOTE: A high period of 0.6uSec and a low period of 1.3uSec is required by both the Proximity
*	and Light Sensors
*                t_low
*   CLDIV =  --------------  - 2
*            2(t_masterclk)
*
*                t_high
*   CHDIV =  --------------  - 2
*            2(t_masterclk)
*
* - Make uC master on the TWI bus
*
*/
void twi0Init(void)
{
	//Setup the TWI mux reset output pin (Experimental on brown robot)
	TWIMUX_RESET_PORT->PIO_OER |= TWIMUX_RESET_PIN;
	TWIMUX_RESET_PORT->PIO_PUER |= TWIMUX_RESET_PIN;
	twiMuxSet;

	REG_PIOA_PER
	|=	PIO_PDR_P3							// Enable PA3 for PIO
	|	PIO_PDR_P4;							// Enable PA4 for PIO
	REG_PIOA_OER
	|=	PIO_PDR_P3							// Enable PA3 as output(TWD0)
	|	PIO_PDR_P4;							// Enable PA4 as output (TWCK0)
	REG_PIOA_PUDR
	|=	PIO_PDR_P3							// Disable PA3 Pullup(TWD0)
	|	PIO_PDR_P4;							// Disable PA4 Pullup (TWCK0)
	REG_PIOA_MDER
	|=	PIO_PDR_P3							// Enable PA3 Open drain(TWD0)
	|	PIO_PDR_P4;							// Enable PA4 Open drain(TWCK0)
	
	
	twi0ClkHigh;
	twi0DataHigh;
}

/*
* Function:
* void twi2Init(void);
*
* Initialises TWI2. Master clock should be setup first. TWI2 is initialised to Slave mode so that
* the LCD interface can requested data from the robot
*
* Inputs:
* none
*
* Returns:
* none
*
* Implementation:
* - Supply master clock to TWI2 peripheral
* - Set PB0(Data) and PB1(Clock) to peripheral B mode (TWI)
* - Set the high and low periods of the TWI clock signal using formula from datasheet
*	NOTE: A high period of 0.6uSec and a low period of 1.3uSec is required by IMU on TWI2
*	1.3uSec = ((x * 2^CKDIV)+4) * 10nSec[100MHz]
*	0.6uSec = ((x * 2^CKDIV)+4) * 10nSec[100MHz]
* - Make uC master on the TWI bus
*
*/
void twi2Init(void)
{
	REG_PMC_PCER0
	|=	(1<<ID_TWI2);						//Enable clock access to TWI2, Peripheral TWI2_ID = 22
	REG_PIOB_PDR
	|=	PIO_PDR_P0							//Enable peripheralB control of PB0 (TWD2)
	|	PIO_PDR_P1;							//Enable peripheralB control of PB1 (TWCK2)
	REG_PIOB_ABCDSR
	|=	PIO_ABCDSR_P0						//Set peripheral B
	|	PIO_ABCDSR_P1;
	twi2Reset;								//Software Reset
	
	REG_TWI2_SMR
	=	TWI_SMR_SADR(TWI2_SLAVE_ADDR);		//Set TWI2 slave address
	
	//TWI2 Clock Waveform Setup. (Ignored for Slave mode, but left in incase Master mode is ever
	//needed)
	REG_TWI2_CWGR
	|=	TWI_CWGR_CKDIV(2)					//Clock speed 400000, fast mode
	|	TWI_CWGR_CLDIV(63)					//Clock low period 1.3uSec
	|	TWI_CWGR_CHDIV(28);					//Clock high period  0.6uSec
	
	REG_TWI2_IER
	=	TWI_IMR_RXRDY;						//Enable the RXRDY interrupt
	
	twi2SlaveMode;							//Slave mode enabled
	
	NVIC_EnableIRQ(ID_TWI2);				//Enable interrupts
}

/*
* Function:
* void twi0MuxReset(void)
*
* Will reset the TWI mux to resolve the dataline being tied low.
*
* Inputs:
* None
*
* Returns:
* None
*
* Implementation:
* Simply pulses the pin connected to the rest line of the mux (pin 3 on mux) off and then on 
* (active low)
*
*/
void twi0MuxReset(void)
{
	twiMuxReset;
	delay_us(100);
	twiMuxSet;
	delay_us(100);
	twi0Reset;

}

/*
* Function:
* uint8_t twi0MuxSwitch(uint8_t channel);
*
* Sets the I2C multiplexer to desired channel.
*
* Inputs:
* uint8_t channel:
*   Mux channel number 0x8 (Ch0) to 0xF (Ch7)
*
* Returns:
* non zero on error
*
* Implementation:
* - Enable TWI0 as bus master
* - Tell TWI0 the address of the Mux
* - Mux has only one internal register, so no need to specify internal address
* - Load channel parameter into the transmit holding register for transmission and set STOP bit
*   (one byte transmission)
* - Wait for transmission to complete before exiting function.
*
*/
uint8_t twi0MuxSwitch(uint8_t channel)
{
	////Event information to be passed to the TWI event logger
	//TwiEvent thisEvent;
	//thisEvent.readOp = 0;
	//thisEvent.slaveAddress = TWI0_MUX_ADDR;
	//thisEvent.regAddress = channel;
	//thisEvent.transferLen = 1;
	//thisEvent.twiBusNumber = 0;
	//thisEvent.timeStamp = sys.timeStamp;
	//thisEvent.bytesTransferred = 1;
	
	twi0bbStart();
	//Send device address with read bit set to 0.
	if(!twi0bbSendByte(TWI0_MUX_ADDR<<1))
	{
		twi0bbStop();
		return 1;
	};
	//Write out the mux channel
	if(!twi0bbSendByte(channel))
	{
		twi0bbStop();
		return 1;
	}
	twi0bbStop();
	return 0;
}

/*
* Function:
* uint8_t twi0ReadMuxChannel(void)
*
* Returns the channel the Mux is currently set to.
*
* Inputs:
* none
*
* Returns:
* A byte that holds a number from 0x8 (Ch0) to 0xF (Ch7) representing the current channel.
* Will return 0x01 on error
*
* Implementation:
* - Enable TWI0 as bus master
* - Tell TWI0 the address of the Mux
* - Mux has only one internal register, so no need to specify internal address
* - Tell TWI0 to read (not write)
* - Set start and stop bits in TWI0 control register (single bit read)
* - Wait for data to be received into RHR
* - Load received data into local variable
* - Wait for transmission to finish before exiting function and returning value
*
*/
uint8_t twi0ReadMuxChannel(void)
{
	uint8_t returnVal;
	twi0bbStart();
	//Send device address with read bit set to 1.
	if(!twi0bbSendByte((TWI0_MUX_ADDR<<1)|1))
	{
		twi0bbStop();
		return 1;
	};
	//Read the mux channel
	if(twi0bbReadByte(&returnVal))
	{
		twi0bbStop();
		return 1;
	}
	twi0bbStop();
	return returnVal;	
}

/*
* Function:
* uint8_t twi0SetCamRegister(uint8_t regAddr)
*
* Function for setting the desired camera register to read from.
*
* Inputs:
* uint8_t regAddr:
*	The address to read from on the camera.
*
* Returns:
* 0 on success, or non zero if there was an error.
*
* Implementation:
* Similar to other write functions in this driver, except that this function will only write out
* a single byte that is the register address. The read operation must be performed with
* twi0ReadCameraRegister() after the desired register has been set.
*
*/
uint8_t twi0SetCamRegister(uint8_t regAddr)
{
	//unsigned int twiStatus = 0;
	//unsigned int twiRetries = 0;
	////Event information to be passed to the TWI event logger
	//TwiEvent thisEvent;
	//thisEvent.readOp = 0;
	//thisEvent.slaveAddress = TWI0_CAM_WRITE_ADDR;
	//thisEvent.regAddress = regAddr;
	//thisEvent.transferLen = 1;
	//thisEvent.twiBusNumber = 0;
	//thisEvent.timeStamp = sys.timeStamp;
	//thisEvent.bytesTransferred = 1;
	
	twi0bbStart();
	//Send device address with read bit set to 0.
	if(!twi0bbSendByte(TWI0_CAM_WRITE_ADDR<<1))
	{
	//	twi0bbStop();
	//	return 1;
	}
	//Write out the mux channel
	if(!twi0bbSendByte(regAddr))
	{
	//	twi0bbStop();
	//	return 1;
	}
	twi0bbStop();
	return 0;

	//thisEvent.operationResult = TWIERR_NONE;
	//twi0LogEvent(thisEvent);
	//return 0;
}

/*
* Function:
* uint8_t twi0ReadCameraRegister(void)
*
* Function for reading a byte from the previously set register on the camera.
*
* Inputs:
* none
*
* Returns:
* The value read from the register.
*
* Implementation:
* Performs a single byte read with no internal address specified. Camera register address must be
* specified with twi0SetCameraRegister()
*
* Improvements:
* Return a 0 on success or a non zero on error. Make register value be returned by a reference
* parameter instead.
*
*/
uint8_t twi0ReadCameraRegister(void)
{

	////Event information to be passed to the TWI event logger
	//TwiEvent thisEvent;
	//thisEvent.readOp = 1;
	//thisEvent.slaveAddress = TWI0_CAM_WRITE_ADDR;
	//thisEvent.regAddress = 0;
	//thisEvent.transferLen = 1;
	//thisEvent.twiBusNumber = 0;
	//thisEvent.timeStamp = sys.timeStamp;
	//thisEvent.bytesTransferred = 1;
		
	uint8_t returnVal;
	twi0bbStart();
	//Send device address with read bit set to 1.
	if(!twi0bbSendByte((TWI0_CAM_WRITE_ADDR<<1)|1))
	{
	//	twi0bbStop();
	//	return 1;
	};
	//Read the mux channel
	if(!twi0bbReadByte(&returnVal))
	{
	//	twi0bbStop();
	//	return 1;
	}
	twi0bbStop();
	return returnVal;
}

/*
* Function: char twiNWrite(unsigned char slave_addr, unsigned char reg_addr,
*								unsigned char length, unsigned char const *data)
*
* Writes bytes out to TWI devices. Allows multiple bytes to be written at once if desired
*
* Inputs:
* slave_addr is the address of the device to be written to on TWIn. reg_addr is the
* 8bit address of the register being written to. length is the number of bytes to be written. *data
* points to the data bytes to be written.
*
* Returns:
* returns 0 on success. otherwise will return 1 on timeout
*
* Implementation:
* Master mode on TWIn is enabled, TWIn is prepared for transmission ie slave and register addresses
* are set and register address size is set to 1 byte. Next, transmission takes place but there are
* slightly different procedures for single and multi byte transmission. On single byte
* transmission, the STOP state is set in the TWI control register immediately after the byte to be
* sent is loaded into the transmission holding register. On multi-byte transmission, the STOP
* flag isn't set until all bytes have been sent and the transmission holding register is clear.
*
*/
char twi0Write(unsigned char slave_addr, unsigned char reg_addr,
					unsigned char length, unsigned char const *data)
{
	//unsigned int twiStatus = 0;
	//unsigned int twiRetries = 0;
	////Event information to be passed to the TWI event logger
	//TwiEvent thisEvent;
	//thisEvent.readOp = 0;
	//thisEvent.slaveAddress = slave_addr;
	//thisEvent.regAddress = reg_addr;
	//thisEvent.transferLen = length;
	//thisEvent.twiBusNumber = 0;
	//thisEvent.timeStamp = sys.timeStamp;

	//Select the slave to write to
	twi0bbStart();
	if(!twi0bbSendByte(slave_addr<<1))
	{
		twi0bbStop();
		return 1;
	}

	if(!twi0bbSendByte(reg_addr))
	{
		twi0bbStop();
		return 1;
	}

	for(uint8_t l = 0; l < length; l++)
	{
		if(!twi0bbSendByte(data[l]))
		{
			twi0bbStop();
			return 1;
		}
	}

	twi0bbStop();

	//thisEvent.operationResult = TWIERR_NONE;
	//twi0LogEvent(thisEvent);
	return 0;
}

char twi2Write(unsigned char slave_addr, unsigned char reg_addr,
				unsigned char length, unsigned char const *data)
{
	if(length == 0)						//Make sure length is valid
		length = 1;
	//note txcomp MUST = 1 before writing (according to datasheet)
	twi2MasterMode;								//Enable master mode
	twi2SetSlave(slave_addr);					//Slave device address
	twi2RegAddrSize(1);							//Set register address length to 1 byte
	twi2RegAddr(reg_addr);						//set register address to write to

	if(length == 1)
	{
		twi2Send(data[0]);						//set up data to transmit
		twi2Stop;								// Send a stop bit
		//while Transmit Holding Register not ready. wait.
		if(waitForFlag(&REG_TWI2_SR, TWI_SR_TXRDY, TWI_TXRDY_TIMEOUT))
			return 1;
	} else {
		for(unsigned char b = 0; b < length; b++)//Send data bit by bit until data length is reached
		{
			twi2Send(data[b]);					//set up data to transmit
			//while Transmit Holding Register not ready. wait.
			if(waitForFlag(&REG_TWI2_SR, TWI_SR_TXRDY, TWI_TXRDY_TIMEOUT))
				return 1;
		}
		twi2Stop;								// Send a stop bit
	}
	//while transmit not complete. wait.
	if(waitForFlag(&REG_TWI2_SR, TWI_SR_TXCOMP, TWI_TXCOMP_TIMEOUT))
		return 1;
	else
		return 0;
}

/*
* Function: char twiNRead(unsigned char slave_addr, unsigned char reg_addr,
*								unsigned char length, unsigned char const *data)
*
* TWI interface read functions. Allows reading multiple bytes sequentially
*
* Inputs:
* slave_addr is the address of the device to be read from on TWIn. The address varies even for the
* IMU driver because the IMU and compass have different TWI slave addresses. reg_addr is the address
* of the register being read from. length is the number of bytes to be read. The IMU automatically
* increments the register address when reading more than one byte. *data points to the location in
* memory where the retrieved data will be stored.
*
* Returns:
* returns 0 on success.
*
* Implementation:
* Master mode on TWIn is enabled, TWIn is prepared for transmission ie slave and register addresses
* are set and register address size is set to 1 byte. Next, reception takes place but there are
* different procedures for single and multi byte reception. On single byte reception, the START and
* STOP flags are set simultaneously in TWIn's control register to indicate that only one byte will
* be read before communication is stopped. With multi-byte reception, the START flag is set
* initially, and the STOP flag in the control register is set when the second to last byte has been
* received (ie there will only be one byte left to receive after the STOP flag is set)
*
*/
char twi0Read(unsigned char slave_addr, unsigned char reg_addr,
					unsigned char length, unsigned char *data)
{
	////Event information to be passed to the TWI event logger
	//TwiEvent thisEvent;
	//thisEvent.readOp = 1;
	//thisEvent.slaveAddress = slave_addr;
	//thisEvent.regAddress = reg_addr;
	//thisEvent.transferLen = length;
	//thisEvent.twiBusNumber = 0;
	//thisEvent.timeStamp = sys.timeStamp;
	
	//Write out register address
	twi0bbStart();
	if(!twi0bbSendByte(slave_addr<<1))
	{
		twi0bbStop();
		return 1;
	}
	if(!twi0bbSendByte(reg_addr))
	{
		twi0bbStop();
		return 1;
	}
	//Repeat start
	twi0bbRepeatStart();
	//Now read data back from that register
	if(!twi0bbSendByte((slave_addr<<1)|1))
	{
		twi0bbStop();
		return 1;
	}
	//Read bytes back
	for(uint8_t l = 0; l < length; l++)
	{
		if(!twi0bbReadByte(&data[l]))
		{
			twi0bbStop();
			return 0;				//NACK indicates end of a multi-byte read
		}
	}

	twi0bbStop();
	return 0;
}

char twi2Read(unsigned char slave_addr, unsigned char reg_addr,
					unsigned char length, unsigned char *data)
{
	if(length == 0)						//Make sure length is valid
		length = 1;
	twi2MasterMode;						//Enable master mode
	twi2SetSlave(slave_addr);			//Slave device address
	twi2SetReadMode;					//Set to read from register
	twi2RegAddrSize(1);					//Register addr byte length (0-3)
	twi2RegAddr(reg_addr);				//set up address to read from
	
	if (length == 1)					//If reading one byte, then START and STOP bits need to be
	//set at the same time
	{
		twi2StartSingle;				//Send START & STOP condition as required (single byte read)
		//while Receive Holding Register not ready. wait.
		if(waitForFlag(&REG_TWI2_SR, TWI_SR_RXRDY, TWI_RXRDY_TIMEOUT))
			return 1;
		data[0] = twi2Receive;			//store data received
		//while transmission not complete. wait.
		if(waitForFlag(&REG_TWI2_SR, TWI_SR_TXCOMP, TWI_TXCOMP_TIMEOUT))
			return 1;
		else
			return 0;
	} else {
		twi2Start;						//Send start bit
		for(unsigned char b = 0; b < length; b++)
		{
			if(waitForFlag(&REG_TWI2_SR, TWI_SR_RXRDY, TWI_RXRDY_TIMEOUT))
			return 1;
			data[b] = twi2Receive;
			if(b == length - 2)
			twi2Stop;					//Send stop on reception of 2nd to last byte
		}
		//while transmit not complete. wait.
		if(waitForFlag(&REG_TWI2_SR, TWI_SR_TXCOMP, TWI_TXCOMP_TIMEOUT))
			return 1;
	}
	return 0;
}

void TWI2_Handler()
{
	if(REG_TWI2_IMR & TWI_IMR_RXRDY)		//If RXRDY interrupt
	{
		sys.comms.twi2ReceivedDataByte = twi2Receive;
		sys.flags.twi2NewData = 1;
	}
}

/*
* Function:
* uint8_t twi0LogEvent(TwiEvent event)
*
* Logs twi0 Events to an array for debugging purposes
*
* Inputs:
* TwiEvent event:
*	Structure that contains information about the event to log
*
* Returns:
* 1 when error occurs on the TWI bus, otherwise returns a 0
*
* Implementation:
* Writes the data stored in event to a global array
*
*/
uint8_t twi0LogEvent(TwiEvent event)
{
	//Shift log entries up
	for(uint8_t i = TWI_LOG_NUM_ENTRIES - 1; i > 0; i--)
		twi0Log[i] = twi0Log[i - 1];
	
	//Store the latest event
	twi0Log[0] = event;
	// == TWIERR_TXCOMP || event.operationResult == TWIERR_TXRDY
	if(event.operationResult)	//If error occurred in the last event
	{
		led2Tog;
		delay_ms(100);
		return 1;				//Put breakpoint here to see errors
	}
	else
		return 0;
}