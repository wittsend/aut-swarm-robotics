/*
* sensor_functions.h
*
* Author : Matthew Witt (pxf5695@autuni.ac.nz)
* Created: 21/09/2017 7:05:43 PM
*
* Project Repository: https://github.com/wittsend/aut-swarm-robotics
*
* Header file for sensor functions
*
* More Info:
* Atmel SAM 4N Processor Datasheet:http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
* Relevant reference materials or datasheets if applicable
*
* Functions:
* void funcName(void)
*
*/

#ifndef SENSOR_FUNCTIONS_H_
#define SENSOR_FUNCTIONS_H_

//////////////[Defines]/////////////////////////////////////////////////////////////////////////////
//Parameter definitions for lcfRetrieveLightData()
#define SF_RGB_ONLY			0
#define SF_RGB_AND_HSV		1

////Hue angle constants (Should help speed up maths)
#define SF_MAX_HUE_ANGLE	360
#define SF_HUE_ANGLE_DIV3	120
#define SF_HUE_ANGLE_DIV6	60

////Proximity sensor array elements
#define SF_PROX_FRONT		0
#define SF_PROX_FRONTL		1
#define SF_PROX_REARL		2
#define SF_PROX_REAR		3
#define SF_PROX_REARR		4
#define SF_PROX_FRONTR		5

//////////////[Type Definitions]////////////////////////////////////////////////////////////////////
//A set of thresholds used to filter out pixels of a certain shade of colour (Used as a parameter
//for sfCamScanForColour()
typedef struct ColourSignature
{
	uint16_t startHue;			//Lower hue(colour) angle threshold (0-359 deg)
	uint16_t endHue;			//Upper hue angle threshold (0-359 deg)
	uint16_t startSaturation;	//Lower saturation threshold (0x0-0xFFFF)
	uint16_t endSaturation;		//Upper saturation threshold (0x0-0xFFFF)
	uint16_t startValue;		//Lower value threshold (0x0-0xFFFF)
	uint16_t endValue;			//Upper value threshold (0x0-0xFFFF)
} ColourSignature;

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
/*
* Function:
* void sfPollSensors(RobotGlobalStructure *sys)
*
* Polls sensors at the desired intervals
*
* Inputs:
* [input arguments and any relevant explanation]
*
* Returns:
* [return values and any relevant explanation]
*
* Implementation:
* [explain key steps of function]
* [use heavy detail for anything complicated]
* Template c file function header. H file function header will be the same without the
* implementation/improvement section
*
* Improvements:
* [Ideas for improvements that are yet to be made](optional)
*
*/
void sfPollSensors(RobotGlobalStructure *sys);

/*
* Function:
* void sfRGB2HSV(struct ColourSensorData *colours)
*
* Converts RGB to HSV and stores them in a ColourSensorData structure
*
* Inputs:
* struct ColourSensorData *colours
*   Pointer to a ColourSensorData structure to store the calculated HSV values
*
* Returns:
* none
*
*/
void sfRGB2HSV(ColourSensorData *colours);

/*
* Function:
* void sfRGB5652HSV(struct ColourSensorData *colours)
*
* Converts RGB565 values to HSV and stores them in a ColourSensorData structure
*
* Inputs:
* struct ColourSensorData *colours
*   Pointer to a ColourSensorData structure to store the calculated HSV values
*
* Returns:
* none
*
*/
void sfRGB5652HSV(struct ColourSensorData *colours);

void sfRGB565Convert(uint16_t pixel, uint16_t *red, uint16_t *green, uint16_t *blue);

float sfCamScanForColour(uint16_t verStart, uint16_t verEnd, uint16_t horStart, uint16_t horEnd,
						ColourSignature sig, float sectionScores[], uint8_t sections, 
						float minScore, float *validPixelScore);

bool sfCheckImagePixel(uint16_t row, uint16_t col, ColourSignature sig);			

#endif /* SENSOR_FUNCTIONS_H_ */