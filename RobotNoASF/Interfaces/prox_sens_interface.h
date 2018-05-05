/*
* prox_sens_interface.h
*
* Author : Esmond Mather and Matthew Witt
* Created: 11/07/2017 10:05:52 AM
*
* Project Repository: https://github.com/wittsend/aut-swarm-robotics
*
* Register and command defines for proximity sensors and function definitions for accessing prox-
* imity sensors
*
* More Info:
* Atmel SAM 4N Processor Datasheet:http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
* APDS-9930 Proximity sensor Datasheet:https://docs.broadcom.com/docs/AV02-3190EN
*
* Register address format:
* Register addresses are only up to 5bits long. The remaining 3bits are used to determine the method
* of data transfer to and from the sensor.
*  bit   7     6     5     4     3     2     1     0
*     +-----+-----+-----+-----+-----+-----+-----+-----+
*     | CMD |    TYPE   |          Address            |
*     +-----+-----+-----+-----+-----+-----+-----+-----+
* CMD should always be 1. TYPE determines the mode of transfer. a value of 0b00 in TYPE means that
* each consecutive read or write to a register address will be to/from that same address. A value
* of 0b01 for TYPE will have the register address increment on each consecutive read/write. This
* simplifies the task of reading long integer values.
*
* Functions:
* void proxSensInit(void)
* uint8_t proxSingleSensInit(uint8_t channel)
* uint16_t proxSensRead(uint8_t channel)
* uint16_t proxAmbRead(uint8_t channel)
* void proxAmbModeEnabled(void)
* void proxModeEnabled(void)
*
*/

#ifndef PROX_SENS_INTERFACE_H_
#define PROX_SENS_INTERFACE_H_

//Distance Threshold values
#define PS_IN_RANGE		0x0070	//Should be prox value when an object is around 100mm away. Value
								//increases as object draws closer. Max is 0x03FF.
#define PS_CLOSEST		0x03FF	//Value from sensor when item is close up

//////////////[Type definitions]////////////////////////////////////////////////////////////////////
typedef enum ProximityMode {PS_PROXIMITY, PS_AMBIENT} ProximityMode;

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
/*
* Function:
* void proxSensInit(void)
*
* Initialise all proximity sensors. Tidies up setup() function
*
* Inputs:
* none
*
* Returns:
* none
*
*/
void proxSensInit(void);

/*
* Function:
* uint8_t proxSingleSensInit(uint8_t channel)
*
* This function will pass the desired channel to the Multiplexer and setup an *individual* proximity
* sensor
*
* Inputs:
* uint8_t channel:
*    The I2C multiplexer channel of the desired proximity sensor
*
* Returns:
* none
*
*/
uint8_t proxSingleSensInit(uint8_t channel);

/*
* Function:
* uint16_t proxSensRead(uint8_t channel)
*
* Retrieves the Proximity Sensor (16-bit) data from the selected sensor.
*
* Inputs:
* uint8_t channel:
*    The I2C multiplexer channel of the desired proximity sensor
*
* Returns:
* 16bit long integer containing the value of the proximity ADC
*
*/
uint16_t proxSensRead(uint8_t channel);

/*
* Function:
* uint16_t proxAmbRead(uint8_t channel)
*
* Retrieves the Ambient light (16-bit) data from the selected proximity sensor.
*
* Inputs:
* uint8_t channel:
*    The I2C multiplexer channel of the desired proximity sensor
*
* Returns:
* 16bit long integer containing the value proportional to lux hitting sensor
*
*/
uint16_t proxAmbRead(uint8_t channel);

/*
* Function:
* ProximityMode proxCurrentMode(void)
*
* Indicates the current sensing mode of the proximity sensors
*
* Inputs:
* none
*
* Returns:
* Returns whether the sensors are in proximity or ambient mode.
*
*/
ProximityMode proxCurrentMode(void);

/*
* Function:
* void proxAmbModeEnabled(void)
*
* Enables ambient light mode and disables proximity mode on the proximity sensors.
*
* Inputs:
* none
*
* Returns:
* 0 When completed
*
*/
uint8_t proxAmbModeEnabled(void);

/*
* Function:
* void proxModeEnabled(void)
*
* Enables proximity detection mode and disables ambient light mode on the proximity sensors.
*
* Inputs:
* none
*
* Returns:
* 0 When completed
*
*/
uint8_t proxModeEnabled(void);
#endif /* PROX_SENS_INTERFACE_H_ */