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
* void sfPollSensors(RobotGlobalStructure *sys)
* void sfRGB2HSV(ColourSensorData *colours);
* void sfRGB5652HSV(struct ColourSensorData *colours);
* void sfRGB565Convert(uint16_t pixel, uint16_t *red, uint16_t *green, uint16_t *blue);
* float sfCamScanForColour(uint16_t verStart, uint16_t verEnd, uint16_t horStart, uint16_t horEnd,
*						ColourSignature sig, float sectionScores[], uint8_t sections,
*						float minScore, float *validPixelScore);
* bool sfCheckImagePixel(uint16_t row, uint16_t col, ColourSignature sig);
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
* RobotGlobalStructure *sys
*	Pointer to the robot global data structure
*
* Returns:
* none
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
* Converts RGB565 values to HSV and stores them in a ColourSensorData structure. Used for converting
* camera pixel data to HSV
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

/*
* Function:
* void sfRGB565Convert(uint16_t pixel, uint16_t *red, uint16_t *green, uint16_t *blue)
*
* Extracts RGB565 data from the raw data words retrieved from the FIFO into separate usable RGB
* values.
*
* Inputs:
* uint16_t pixel:
*	unsigned 16 bit integer containing the RGB565 data.
* uint16_t *red:
*	Pointer to the integer where the red channel data will be stored
* uint16_t *green:
*	Pointer to the integer where the green channel data will be stored
* uint16_t *blue:
*	Pointer to the integer where the blue channel data will be stored
*
* Returns:
* None.
*
*/
void sfRGB565Convert(uint16_t pixel, uint16_t *red, uint16_t *green, uint16_t *blue);

/*
* Function:
* float sfCamScanForColour(uint16_t verStart, uint16_t verEnd, uint16_t horStart, uint16_t horEnd,
*							ColourSignature sig, float sectionScores[], uint8_t sections,
*							float minScore, float *validPixelScore)
*
* Scans a portion of the image in the FIFO for a specified section of the picture. Returns an array
* with weighted scores indicating where in the picture the desired colour appears. Used for tracking
* the position of a colour with the camera
*
* Inputs:
* uint16_t verStart:
*	Sets the bounds of the area in the image to be scanned
* uint16_t verEnd:
*	Sets the bounds of the area in the image to be scanned
* uint16_t horStart:
*	Sets the bounds of the area in the image to be scanned
* uint16_t horEnd:
*	Sets the bounds of the area in the image to be scanned
* ColourSignature sig:
*	Holds the thresholds of the colour to be scanned for (in HSV)
* float sectionScores[]:
*	An array that returns the weighted scores of the position of the colour (horizontally)
* uint8_t sections:
*	The number of sections to divide the image up into (must be an even number)
* float minScore:
*	The minimum score that must be achieved over all sections to achieve a valid scan
*	(fraction of 1)
* float *validPixelScore:
*	The number of pixels found in the given area that meet the threshold requirements
*
* Returns:
* Returns a single score between -1 and 1 that indicates the direction that the colour is in (a
* negative number means to the left, and positive to the right)
*
*/
float sfCamScanForColour(uint16_t verStart, uint16_t verEnd, uint16_t horStart, uint16_t horEnd,
						ColourSignature sig, float sectionScores[], uint8_t sections, 
						float minScore, float *validPixelScore);

/*
* Function:
* bool sfCheckImagePixel(uint16_t row, uint16_t col, ColourSignature sig)
*
* Checks if the specified pixel in the camera FIFO meets the threshold requirements
*
* Inputs:
* uint16_t row:
*	Row or Y position of the pixel to be checked
* uint16_t col:
*	Column or X position of the pixel to be checked
* ColourSignature sig:
*	Colour thresholds to check for.
*
* Returns:
* Boolean value (true or false) if the pixel was within the threshold or not.
*
*/
bool sfCheckImagePixel(uint16_t row, uint16_t col, ColourSignature sig);			

#endif /* SENSOR_FUNCTIONS_H_ */