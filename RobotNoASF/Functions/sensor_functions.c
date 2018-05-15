/*
* sensor_functions.c
*
* Author : Matthew Witt (pxf5695@autuni.ac.nz)
* Created: 21/09/2017 7:05:30 PM
*
* Project Repository: https://github.com/wittsend/aut-swarm-robotics
*
* Handles polling of sensors
*
* More Info:
* Atmel SAM 4N Processor Datasheet:http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
* Relevant reference materials or datasheets if applicable
*
* Functions:
* void funcName(void)
*
*/

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "../robot_setup.h"

#include "../Interfaces/adc_interface.h"
#include "../Interfaces/line_sens_interface.h"
#include "../Interfaces/prox_sens_interface.h"
#include "../Interfaces/light_sens_interface.h"
#include "../Interfaces/twimux_interface.h"
#include "../Interfaces/camera_buffer_interface.h"

#include "sensor_functions.h"

//////////////[Private Functions]///////////////////////////////////////////////////////////////////
static void sfUpdateProxStatus(RobotGlobalStructure *sys);
static void sfGetProxSensorData(RobotGlobalStructure *sys);
static uint8_t sfUpdateLineSensorStates(RobotGlobalStructure *sys);
static void sfGetLineDirection(RobotGlobalStructure *sys);
static uint8_t sfLightCapture(uint8_t channel, ColourSensorData *colours);
//////////////[Functions]///////////////////////////////////////////////////////////////////////////
/*
* Function:
* void sfPollSensors(RobotGlobalStructure *sys)
*
* Polls sensors at the desired intervals
*
* Inputs:
* RobotGlobalStructure *sys
*   Pointer to the global robot data structure
*
* Returns:
* none
*
* Implementation:
* As with the other polling functions, checks if the correct amount of time has elapsed, then
* updates the data in the global structure
*
*/
void sfPollSensors(RobotGlobalStructure *sys)
{
	//When to next poll each set of sensors
	static uint32_t proxNextTime = 0;
	static uint32_t colourNextTime = 0;
	static uint32_t lineNextTime = 0;
	
	//Update Prox sensor Status and SetMode
	sfUpdateProxStatus(sys);
	
	//Poll prox sensors
	if(sys->timeStamp > proxNextTime && sys->sensors.prox.pollEnabled 
	&& sys->sensors.prox.status != PS_NOT_READY)
	{
		proxNextTime = sys->timeStamp + sys->sensors.prox.pollInterval;
		sfGetProxSensorData(sys);
	}

	//Poll colour sensors
	if(sys->timeStamp > colourNextTime && sys->sensors.colour.pollEnabled)
	{
		colourNextTime = sys->timeStamp + sys->sensors.colour.pollInterval;
		if(sys->sensors.colour.pollEnabled & 0x02)
			sfLightCapture(MUX_LIGHTSENS_L, &sys->sensors.colour.left);
		if(sys->sensors.colour.pollEnabled & 0x01)
			sfLightCapture(MUX_LIGHTSENS_R, &sys->sensors.colour.right);
		if(sys->sensors.colour.getHSV)
		{
			sfRGB2HSV(&(sys->sensors.colour.left));				//Derive HSV figures from RGB
			sfRGB2HSV(&(sys->sensors.colour.right));			//Derive HSV figures from RGB
		}
	}

	//Poll line sensors
	if(sys->timeStamp > lineNextTime && sys->sensors.line.pollEnabled)
	{
		lineNextTime = sys->timeStamp + sys->sensors.line.pollInterval;
		sfGetLineDirection(sys);
	}
}

/*
* Function:
* void sfGetProxSensorData(RobotGlobalStructure *sys)
*
* Retrieves data from the proximity sensors and stores it in the global data structure
*
* Inputs:
* RobotGlobalStructure *sys
*	Pointer to the robot global data structure
*
* Returns:
* none
*
* Implementation:
* Retrieves data one by one from each sensor and stores it in the structure using a for loop.
*
*/
void sfGetProxSensorData(RobotGlobalStructure *sys)
{
	for(uint8_t sensor = MUX_PROXSENS_A; sensor > 0; sensor++)
	{
		if(sys->sensors.prox.pollEnabled & (1<<(sensor - 0xFA)))
		{
			switch(sys->sensors.prox.status)
			{
				case PS_PROXIMITY:
					sys->sensors.prox.sensor[sensor - 0xFA] = proxSensRead(sensor);
					break;
					
				case PS_AMBIENT:
					sys->sensors.prox.sensor[sensor - 0xFA] = proxAmbRead(sensor);
					break;
					
				case PS_NOT_READY:
					//Do Nothing
					break;
			}
			
		}
	}
}

/*
* Function:
* sfUpdateProxStatus(RobotGlobalStructure *sys)
*
* Updates the status of the proximity sensors, and allows the changing of the proximity sensor's
* light sensing mode between proximity (IR) and ambient light. Needs to happen much faster than
* the prox sensor poll rate, so it has it's own function
*
* Inputs:
* RobotGlobalStructure *sys
*	Pointer to the robot global data structure
*
* Returns:
* none
*
* Implementation:
* TODO: Implementation Description
*
*/
void sfUpdateProxStatus(RobotGlobalStructure *sys)
{
	sys->sensors.prox.status = proxCurrentMode();
	
	//If the prox sensors aren't currently in the process of changing modes, and the set Mode is not
	//equal to the current mode
	if((sys->sensors.prox.status != PS_NOT_READY
	&& ( sys->sensors.prox.status != sys->sensors.prox.setMode))
	|| sys->sensors.prox.status == PS_NOT_READY)
	{
		//Then change the mode of the prox sensors
		switch(sys->sensors.prox.setMode)
		{
			case PS_AMBIENT:
				proxAmbModeEnabled();
						break;
			
			case PS_PROXIMITY:
				proxModeEnabled();
						break;
			
			case PS_NOT_READY:
				//Illegal state, make setMode the same as status
				if(sys->sensors.prox.status != PS_NOT_READY)
				sys->sensors.prox.setMode = sys->sensors.prox.status;
				break;
		}
		
		sys->sensors.prox.status = proxCurrentMode();
	}
}

/*
* Function:
* void sfGetLineDirection(RobotGlobalStructure *sys)
*
* This function examines the states of the line follower sensors and determines the direction and
* urgency factor by which the robot should move to find its way to the centre of the line.
*
* Inputs:
* none
*
* Returns:
* returns a signed integer between -3 and 3 that determines the direction and speed magnitude that
* the robot should move to find the centre of the line.
* A negative output means that the robot should move left to find the line and a positive output
* means that the robot should move right. 0 means keep going straight because no direction data is
* able to be derived from sensor array.
*
* Implementation:
* All the line sensor states are loaded into a single byte so that they can easily managed by a
* switch statement rather than a series of unwieldy if statements. A table describing the states
* and there digital values can be seen below:
*
* Sensor array state descriptions:
* _______________________________________________________________________________________
*| State  | Outer left | Inner left | Inner right | Outer right | Function Output value: |
*|________|_____8______|_____4______|______2______|______1______|________________________|
*|__0x0___|____________|____________|_____________|_____________|__________0_____________|
*|__0x8___|_____X______|____________|_____________|_____________|_________-3_____________|
*|__0xC___|_____X______|_____X______|_____________|_____________|_________-2_____________|
*|__0xE___|_____X______|_____X______|______X______|_____________|_________-1_____________|
*|__0xF___|_____X______|_____X______|______X______|______X______|__________0_____________|
*|__0x7___|____________|_____X______|______X______|______X______|__________1_____________|
*|__0x3___|____________|____________|______X______|______X______|__________2_____________|
*|__0x1___|____________|____________|_____________|______X______|__________3_____________|
*|__0x2___|____________|____________|______X______|_____________|__________1_____________|
*|__0x4___|____________|_____X______|_____________|_____________|_________-1_____________|
*|__0x6___|____________|_____X______|______X______|_____________|__________0_____________|
*
*/
void sfGetLineDirection(RobotGlobalStructure *sys)
{
	//Get updated sensor data
	sfUpdateLineSensorStates(sys);
	//Combine sensor states from sensor structure into single byte that can be used by a switch
	//statement. See above for description of each state.
	uint8_t sensorStates
	=	(sys->sensors.line.outerLeft<<3)	//Outer left has binary weighting 8
	|	(sys->sensors.line.innerLeft<<2)	//Inner left has binary weighting 4
	|	(sys->sensors.line.innerRight<<1)	//Inner right has binary weighting 2
	|	(sys->sensors.line.outerRight<<0);	//Outer right has binary weighting 1
	
	switch(sensorStates)
	{
		case 0x0:		//Straight, no line
			sys->sensors.line.direction = 0;
			sys->sensors.line.detected = 0;
			return;
					
		case 0x8:		//Move left by factor 3
			sys->sensors.line.direction = -3;
			sys->sensors.line.detected = 1;
			return;
			
		case 0xC:		//Move left by factor 2
			sys->sensors.line.direction = -2;
			sys->sensors.line.detected = 1;
			return;

		case 0xE:		//Move left by factor 1
			sys->sensors.line.direction = -1;
			sys->sensors.line.detected = 1;
			return;

		case 0xF:		//Straight, line
			sys->sensors.line.direction = 0;
			sys->sensors.line.detected = 1;
			return;

		case 0x7:		//Move right by factor 1
			sys->sensors.line.direction = 1;
			sys->sensors.line.detected = 1;
			return;

		case 0x3:		//Move right by factor 2
			sys->sensors.line.direction = 2;
			sys->sensors.line.detected = 1;
			return;

		case 0x1:		//Move right by factor 3
			sys->sensors.line.direction = 3;
			sys->sensors.line.detected = 1;
			return;

		case 0x2:		//Move right by factor 1
			sys->sensors.line.direction = 1;
			sys->sensors.line.detected = 1;
			return;

		case 0x4:		//Move left by factor 1
			sys->sensors.line.direction = -1;
			sys->sensors.line.detected = 1;
			return;

		case 0x6:		//Straight, line
			sys->sensors.line.direction = 0;
			sys->sensors.line.detected = 1;
			return;
	}
}

/*
* Function:
* void sfUpdateLineSensorStates(RobotGlobalStructure *sys)
*
* Sees if any sensors have made a definite state change and loads the states into the line sensor
* state structure for use by other functions in this module.
*
* Inputs:
* none
*
* Returns:
* 1 if line state change detected, otherwise 0
* 2 if wrong robot selected
*
* Implementation:
* - Read state value of sensor.
* - if a state other than NO_CHANGE is returned then update the value stored in the line sensor
*   data structure for the given sensor.
* - Repeat until all four sensors have been read.
*
*/
uint8_t sfUpdateLineSensorStates(RobotGlobalStructure *sys)
{
	uint8_t sensorValue;							//Temporarily stores state of a single sensor
	uint8_t returnVal = 0;							//Returns non 0 if any sensor has changed state
	
	sensorValue = lfLineDetected(LF_OUTER_L);		//Look for line on outer left sensor
	if (sensorValue != NO_CHANGE)					//If this sensor has changed state
	sys->sensors.line.outerLeft = sensorValue;		//Update line follower data structure with new
													//data
	if(sensorValue == LINE)
		returnVal = 1;
	
	sensorValue = lfLineDetected(LF_INNER_L);		//Look for line on inner left sensor
	if (sensorValue != NO_CHANGE)					//If this sensor has changed state
		sys->sensors.line.innerLeft = sensorValue;	//Update line follower data structure with new
													//data
	if(sensorValue == LINE)
		returnVal = 1;
	
	sensorValue = lfLineDetected(LF_OUTER_R);		//Look for line on outer right sensor
	if (sensorValue != NO_CHANGE)					//If this sensor has changed state
		sys->sensors.line.outerRight = sensorValue;	//Update line follower data structure with new
													//data
	if(sensorValue == LINE)
		returnVal = 1;
	
	sensorValue = lfLineDetected(LF_INNER_R);		//Look for line on inner right sensor
	if (sensorValue != NO_CHANGE)					//If this sensor has changed state
		sys->sensors.line.innerRight = sensorValue;	//Update line follower data structure with new
													//data
	if(sensorValue == LINE)
		returnVal = 1;
	return returnVal;
}

/*
* Function:
* uint8_t sfLightCapture(uint8_t channel, uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *w)
*
* Retrieves the (16-bit) light data of all colours from the selected Light Sensor (RGB and HSV)
*
* Inputs:
* struct ColourSensorData *colours
*   Pointer to a ColourSensorData structure, which is where the output will be stored
*
* Returns:
* Always 0
*
* Implementation:
* Uses lightSensRead() to read the values from each colour channel on the given colour sensor and
* loads the data into the ColourSensorData structure provided in the parameters.
* After that, the ColourSensorData structure is passed to lcfRGB2HSV to find the hue,
* saturation and value figures for the retrieved RGB values and stores them in ColourSensorData
* structure.
*/
uint8_t sfLightCapture(uint8_t channel, ColourSensorData *colours)
{
	uint8_t returnVal = 0;

	colours->red = lightSensRead(channel, LS_RED_REG);		//Read red channel
	colours->green = lightSensRead(channel, LS_GREEN_REG);	//Read green channel
	colours->blue = lightSensRead(channel, LS_BLUE_REG);	//Read blue channel
	colours->white = lightSensRead(channel, LS_WHITE_REG);	//Read white channel
	
	return returnVal;
}

//Calculate just Value from RGB565
unsigned short sfRGB5652V(struct ColourSensorData *colours)
{
	unsigned short rgbMax = 0;
	
	//Find maximum colour channel value (rgbMax)
	if(colours->red > rgbMax) rgbMax = colours->red;
	if((colours->green >> 1) > rgbMax) rgbMax = (colours->green >> 1);
	if(colours->blue > rgbMax) rgbMax = colours->blue;
	
	//Set Value figure to maximum rgb channel figure
	colours->value = rgbMax;
	return rgbMax;
}

//Calculate just Saturation from RGB565. (Must have value first)
unsigned short sfRGB5652S(struct ColourSensorData *colours)
{
	//If value is 0, then no saturation.
	if(colours->value == 0)
	{
		colours->saturation = 0;
		colours->hue = 0;
		return 0;
	}

	const int MAX_RGB555_VAL = 0x1F;
	
	//RGB minimum and maximum
	unsigned short rgbMin = MAX_RGB555_VAL;
		
	//Find minimum colour channel value (rgbMin)
	if(colours->red < rgbMin) rgbMin = colours->red;
	if((colours->green >> 1) < rgbMin) rgbMin = (colours->green >> 1);
	if(colours->blue < rgbMin) rgbMin = colours->blue;
	
	//Calculate saturation
	colours->saturation = (short)((MAX_RGB555_VAL*(colours->value - rgbMin))/colours->value);
	return rgbMin;	
}

//Calculate Hue angle from RGB565. (Must've calculated Value and rgbMin first)
void sfRGB5652H(struct ColourSensorData *colours, unsigned short rgbMin)
{
	//used for hue angle calculation
	int rawHue = 0;
	
	//Calculate Hue angle
	if (colours->value == colours->red)
	rawHue = 0 + SF_HUE_ANGLE_DIV6*((colours->green >> 1) - colours->blue)/(colours->value - rgbMin);
	else if (colours->value == (colours->green >> 1))
	rawHue = SF_HUE_ANGLE_DIV3 + SF_HUE_ANGLE_DIV6
	*(colours->blue - colours->red)/(colours->value - rgbMin);
	else
	rawHue = 2*SF_HUE_ANGLE_DIV3 + SF_HUE_ANGLE_DIV6
	*(colours->red - (colours->green >> 1))/(colours->value - rgbMin);

	//Wrap rawHue to the range 0-360 and store in colours.hue
	while(rawHue < 0) rawHue += 360;
	while(rawHue > 360) rawHue -= 360;
		
	colours->hue = rawHue;

	return;
}

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
* Implementation:
* See
* http://www.rapidtables.com/convert/color/rgb-to-hsv.htm
* https://en.wikipedia.org/wiki/Hue#Computing_hue_from_RGB
*
*/
void sfRGB2HSV(struct ColourSensorData *colours)
{
	//RGB minimum and maximum
	unsigned short rgbMin = MAX_LIGHT_CHANNEL_VAL;
	unsigned short rgbMax = 0;
	
	//used for hue angle calculation
	int rawHue = 0;
	
	//Find maximum colour channel value (rgbMax)
	if(colours->red > rgbMax)
		rgbMax = colours->red;
	if(colours->green > rgbMax)
		rgbMax = colours->green;
	if(colours->blue > rgbMax)
		rgbMax = colours->blue;

	//Find minimum colour channel value (rgbMin)
	if(colours->red < rgbMin)
		rgbMin = colours->red;
	if(colours->green < rgbMin)
		rgbMin = colours->green;
	if(colours->blue < rgbMin)
		rgbMin = colours->blue;
	
	//Set Value figure to maximum rgb channel figure
	colours->value = rgbMax;
	
	//If HSV value equals 0 then we are looking at pure black (no hue or saturation)
	if (colours->value == 0)
	{
		colours->hue = 0;
		colours->saturation = 0;
		return;
	}
	
	//Calculate saturation
	colours->saturation = (short)(MAX_LIGHT_CHANNEL_VAL*(rgbMax - rgbMin)/colours->value);
	
	//If no saturation then we are looking at a perfectly grey item (no hue)
	if (colours->saturation == 0)
	{
		colours->hue = 0;
		return;
	}

	//Calculate Hue angle
	if (rgbMax == colours->red)
		rawHue = 0 + SF_HUE_ANGLE_DIV6*(colours->green - colours->blue)/(rgbMax - rgbMin);
	else if (rgbMax == colours->green)
		rawHue = SF_HUE_ANGLE_DIV3 + SF_HUE_ANGLE_DIV6
					*(colours->blue - colours->red)/(rgbMax - rgbMin);
	else
		rawHue = 2*SF_HUE_ANGLE_DIV3 + SF_HUE_ANGLE_DIV6
					*(colours->red - colours->green)/(rgbMax - rgbMin);

	//Wrap rawHue to the range 0-360 and store in colours.hue
	while(rawHue < 0)
		rawHue += 360;
	while(rawHue > 360)
		rawHue -= 360;
	colours->hue = rawHue;

	return;
}

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
* Implementation:
* See
* http://www.rapidtables.com/convert/color/rgb-to-hsv.htm
* https://en.wikipedia.org/wiki/Hue#Computing_hue_from_RGB
*
*/
void sfRGB5652HSV(struct ColourSensorData *colours)
{
	unsigned short rgbMin;
	sfRGB5652V(colours);
	if(colours->value) rgbMin = sfRGB5652S(colours);
	if(colours->saturation) sfRGB5652H(colours, rgbMin);
}

//TODO: Commenting
void sfRGB565Convert(uint16_t pixel, uint16_t *red, uint16_t *green, uint16_t *blue)
{	
	/*******Version 1********/
	//float fRed, fGreen, fBlue;
	//////Converts 16-bit RGB565 pixel data to RGB values
	//fRed = ((float)((pixel & 0xF800) >> 11)/0x1F)*0xFFFF;
	//fGreen = ((float)((pixel & 0x07E0) >> 5)/0x3F)*0xFFFF;
	//fBlue = ((float)((pixel & 0x001F) >> 0)/0x1F)*0xFFFF;
	//
	//*red = (uint16_t)fRed;
	//*green = (uint16_t)fGreen;
	//*blue = (uint16_t)fBlue;
	
	/********Version 2*******/
	////Converts 16-bit RGB565 pixel data to RGB161616 values
	//*red = (uint16_t)((pixel & 0xF800) >> 11)*(0xFFFF/0x1F);
	//*green = (uint16_t)((pixel & 0x07E0) >> 5)*(0xFFFF/0x3F);
	//*blue = (uint16_t)((pixel & 0x001F) >> 0)*(0xFFFF/0x1F);		
	
	/********Version 3*******/
	//Converts 16-bit RGB565 pixel data to RGB565 values
	*red = (uint16_t)((pixel & 0xF800) >> 11);
	*green = (uint16_t)((pixel & 0x07E0) >> 5);
	*blue = (uint16_t)((pixel & 0x001F) >> 0);		
	
}

//TODO: Commenting
float sfCamScanForColour(uint16_t verStart, uint16_t verEnd, uint16_t horStart, uint16_t horEnd, 
						ColourSignature sig, float sectionScores[], uint8_t sections, float minScore)
{
	//This function requires and even number of sections.
	if(sections%2) return 1;
	uint16_t sectionScoresPix[sections];//The section scores as pixel counts

	uint32_t sectionWidth = CAM_IMAGE_WIDTH/sections;//The width of each ection in pixels
	int maxSectionVal = 0;		//The maximum score of all sections (used for normalisation)
	int validPixelCount = 0;	//The total number of valid pixels found 
	float finalScore = 0;		//The final directionally weighted score
	
	float weight = -sections + sections/2; //Weight used to calculate final directional score
	
	//Check parameters are valid
	if(verEnd > CAM_IMAGE_HEIGHT) verEnd = CAM_IMAGE_HEIGHT;
	if(verStart > verEnd) verStart = verEnd;
	if(horEnd > CAM_IMAGE_WIDTH) horEnd = CAM_IMAGE_WIDTH;
	if(horStart > horEnd) horStart = horEnd;
	
	//Make sure the score tables are clear
	for(uint8_t i = 0; i < sections; i++) 
	{
		sectionScoresPix[i] = 0;	//Section pixel quantity scores
		sectionScores[i] = 0.0;		//Normalised Scores
	}

	//For each line
	for(uint16_t thisLine = verStart; thisLine <= verEnd; thisLine++)
	{
		//For each pixel in each line.
		for(uint16_t thisPixel = horStart; thisPixel <= horEnd; thisPixel++)
		{
			if(sfCheckImagePixel(thisLine, thisPixel, sig))
			{
				sectionScoresPix[(int)(thisPixel/sectionWidth)] += 1;
				validPixelCount++;
			}
		}
	}
	
	//If we have found a percentage of pixels greater than the given threshold, then we have the
	//item of interest in front of us, so calculate a directional bias to use to get the robot to
	//face the colour. If we haven't seen enough pixels, then just return a finalScore that is
	//greater than 1 to indicate that we have not found what we are looking for.
	if((float)validPixelCount/((verEnd - verStart)*(horEnd - horStart)) > minScore)
	{
		//Find the maximum in sectionScores[] for normalisation
		for(int i = 0; i < sections; i++)
		{
			if(sectionScoresPix[i] > maxSectionVal) maxSectionVal = sectionScoresPix[i];
		}
	
		//Normalise sectionScores and calculate final score
		for(int i = 0; i < sections; i++)
		{
			//Normalise
			sectionScores[i] = (float)sectionScoresPix[i]/(float)maxSectionVal;
			//If the current section score is greater than the minimum percentage of pixels, then add the
			//current score multiplied by the weighting to the finalscore
			finalScore = finalScore + (weight*sectionScores[i]);
			//Increment the weighting as we move across the picture (If there are 6 sections then
			//weighting would go [-3, -2, -1, 1, 2, 3]
			weight += 1;
			if(weight == 0) weight++;
		}
	
		//Divide by the number of sections to get the mean of the score. The absolute final score should
		//never be greater than one if it is legitimate.
		finalScore /= sections;
	
		return finalScore;		
	}
	return 2;
}

//TODO: Comment header. Checks if the desired pixel in the cam FIFO meets the threshold requirements
bool sfCheckImagePixel(uint16_t row, uint col, ColourSignature sig)
{
	ColourSensorData pixel;		//Processed pixel data
	unsigned short rgbMin;		//Needed for sfRGB5652H()
	uint16_t rawPixel;			//Raw pixel data straight from the camera FIFO
	
	//Check parameters are valid
	if(row > CAM_IMAGE_HEIGHT || col > CAM_IMAGE_WIDTH) return false;
	
	//Reading and then processing one pixel at a time seems to be faster than reading one
	//line and then processing one line.
	if(camBufferReadPixel(col, row, &rawPixel)) return false;
	sfRGB565Convert(rawPixel, &pixel.red, &pixel.green, &pixel.blue);
	//sfRGB5652HSV(&pixel);
			
	//Check value first, as thats the easiest calculation
	sfRGB5652V(&pixel);
	//See that the current pixel falls within the desired thresholds
	if(pixel.value >= sig.startValue && pixel.value <= sig.endValue)
	{
		//Next do saturation as thats the next intensive task
		rgbMin = sfRGB5652S(&pixel);
		if(pixel.saturation >= sig.startSaturation && pixel.saturation <= sig.endSaturation)
		{
			//Finally, if Value and Sat are within the threshold, do Hue
			sfRGB5652H(&pixel, rgbMin);
			//If the hue range does not contain the 359->0 degree crossing
			if(sig.startHue <= sig.endHue)
			{
				if(pixel.hue >= sig.startHue && pixel.hue <= sig.endHue) return true;
			} else {
				if((pixel.hue >= sig.startHue && pixel.hue <= 359)
				|| (pixel.hue <= sig.endHue && pixel.hue >= 0))
				{
					return true;
				}
			}
		}
	}
	return false;
}