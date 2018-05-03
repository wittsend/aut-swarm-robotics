/*
* docking_functions.c
*
* Author : Matthew Witt (pxf5695@autuni.ac.nz)
* Created: 25/07/2017 8:07:50 PM
*
* Project Repository: https://github.com/wittsend/aut-swarm-robotics
*
* Contains the docking routine state machine and functions that are useful for docking...
*
* More Info:
* Atmel SAM 4N Processor Datasheet:http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
* Relevant reference materials or datasheets if applicable
*
* Functions:
* uint8_t dfDockWithLightSensor(void)
* uint8_t dfDockWithCamera(RobotGlobalStructure *sys);
* uint8_t dfFollowLine(uint8_t speed, float *lineHeading, RobotGlobalStructure *sys)
* uint8_t dfScanBrightestLightSource(int16_t *brightestHeading)
* float dfScanBrightestLightSourceProx(void);
*
*/

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "../robot_setup.h"

#include "../Interfaces/prox_sens_interface.h"
#include "../Interfaces/twimux_interface.h"			//For Prox sensor TWI defines
#include "../Interfaces/fc_interface.h"
#include "../Interfaces/timer_interface.h"
#include "../Interfaces/motor_driver.h"
#include "../Interfaces/pio_interface.h"
#include "../Interfaces/camera_interface.h"			//For camera based docking
#include "../Interfaces/camera_buffer_interface.h"

#include "motion_functions.h"
#include "navigation_functions.h"
#include "sensor_functions.h"
#include "docking_functions.h"

#include <stdlib.h>				//abs() function in dfFollowLine()

//////////////[Defines]/////////////////////////////////////////////////////////////////////////////
//Docking with Camera Constants
////Scan for Dock Constants. These are thresholds used for detecting the dock with the 
////sfCamScanForColour() function.
#define DCS_SFD_START_LINE		110		//Start horizontal line of the area to be scanned
#define DCS_SFD_END_LINE		130		//End horizontal line
#define DCS_SFD_MIN_PIXELS		50		//A section must contain at least this many pixels of the
										//correct colour before it will be considered.
#define DCS_SFD_SECTIONS		3		//Number of sections to divide up the fetched image strip

//////////////[Global Variables]////////////////////////////////////////////////////////////////////
//This colour signature defines the colour that is expected to be seen on the camera when the 
//docking station is in front of the robot.
ColourSignature dockingStationSig =
{
	.startHue			= 145,
	.endHue				= 160,
	.startSaturation	= 24627,
	.endSaturation		= 0xFFFF,
	.startValue			= 10000,
	.endValue			= 0xFFFF
};

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
/*
* Function:
* void dfDockWithLightSensor(void)
*
* Function to guide the robot to the dock.
*
* Inputs:
* RobotGlobalStructure *sys
*   Pointer to the sys->pos. structure
*
* Returns:
* 0 when docking complete, otherwise non-zero
*
* Implementation:
* The docking function is a state machine that will change states after each step that is required
* for docking is performed. More to come [WIP]
*
* Improvements:
* [Ideas for improvements that are yet to be made](optional)
*
*/
uint8_t dfDockWithLightSensor(RobotGlobalStructure *sys)
{
	static float bHeading = 0;			//Brightest Heading
	static uint8_t lineFound = 0;		//Whether or not we have found the line
	static uint32_t lineLastSeen = 0;	//Time at which line was last detected
	static FDelayInstance delay;		//Delay instance to use with fdelay_ms()
	
	switch(sys->states.dockingLight)
	{
		//Begin by scanning for the brightest light source
		case DS_START:
			//Initialise the required sensors
			sys->sensors.prox.pollEnabled = 0x3F;	//All prox
			sys->sensors.prox.pollInterval = 100;
			sys->sensors.colour.pollEnabled = 0x03;	//All colour
			sys->sensors.colour.pollInterval = 40;
			sys->sensors.line.pollEnabled = 1;		//Line sensors
			sys->sensors.line.pollInterval = 100;	
			
			if(lineFound)
			{
				lineLastSeen = sys->timeStamp;
				bHeading = sys->pos.facing + dfScanBrightestLightSourceProx();
				//if(!dfScanBrightestLightSource(&bHeading, 200, sys))
					sys->states.dockingLight = DS_FACE_BRIGHTEST;
			} else {
				bHeading = sys->pos.facing + dfScanBrightestLightSourceProx();
				//if(!dfScanBrightestLightSource(&bHeading, 359, sys))
					sys->states.dockingLight = DS_FACE_BRIGHTEST;
							
			}
			
			break;
		
		//Turn to face brightest light source seen
		case DS_FACE_BRIGHTEST:
			if(!mfRotateToHeading(bHeading, sys))
			{
				if(lineFound)
					sys->states.dockingLight = DS_FOLLOW_LINE;
				else
					sys->states.dockingLight = DS_MOVE_FORWARD;
			}
			break;
		
		//Move towards brightestes light source
		case DS_MOVE_FORWARD:
			mfTrackLight(70, sys);
			if(!fdelay_ms(&delay, 3400))			//After 3.7 seconds, look for LEDs again
				sys->states.dockingLight = DS_RESCAN_BRIGHTEST;

			if(sys->sensors.line.detected)
			{
				lineFound = 1;
				sys->states.dockingLight = DS_START;
			}
			break;
			
		//Check again for brightest light source by scanning a 180 degree arc left to right to see
		//if we are still on track to find brightest light source
		case DS_RESCAN_BRIGHTEST:
			//Only look in front, because we should still be roughly in the right direction
			bHeading = sys->pos.facing + dfScanBrightestLightSourceProx();
			//if(!dfScanBrightestLightSource(&bHeading, 270, sys))
				sys->states.dockingLight = DS_FACE_BRIGHTEST;
			break;
		
		//Follow the line until an obstacle is encountered
		case DS_FOLLOW_LINE:
			//Enable fast scharge chip polling
			sys->power.pollChargingStateEnabled = 1;
			sys->power.pollChargingStateInterval = 150;
			
			mfTrackLight(45 - (sys->sensors.prox.sensor[SF_PROX_FRONT]*45/1023) + 10, sys);
			if(sys->sensors.prox.sensor[SF_PROX_FRONT] >= PS_CLOSEST)
				sys->states.dockingLight = DS_CHRG_CONNECT;

			if(sys->sensors.line.detected)
				lineLastSeen = sys->timeStamp;
				
			if((sys->timeStamp - lineLastSeen) > 2500) //If line hasn't been detected for 2 seconds,
			{	
				lineFound = 0;									
				sys->states.dockingLight = DS_MOVE_FORWARD;
			}
			break;
		
		//Drive straight ahead until a connection with the charger is connected. When connection
		//is established, exit with a FINISH state. Still need to include a timeout, incase the
		//in front of the robot isn't the charger
		case DS_CHRG_CONNECT:
			//If power connected to fc chip
			if(sys->power.fcChipStatus == FC_STATUS_BF_STAT_INRDY 
			|| sys->power.fcChipStatus == FC_STATUS_BF_STAT_CHRGIN)
			{
				sys->states.dockingLight = DS_FINISHED;	//Docking is complete
				mfStopRobot(sys);					//Stop moving
			} else
				moveRobot(0, 100, 0);
			break;
		
		//If charger hasn't been found after time period, we enter this state. The resulting
		//return value that this state invokes will prompt the caller to avoid an obstacle, or
		//try docking again.
		case DS_CHRG_NOT_FOUND:
			lineFound = 0;
			sys->states.dockingLight = DS_START;
			break;
		
		case DS_FINISHED:
			lineFound = 0;
			sys->states.dockingLight = DS_START;
			break;
	}
	return sys->states.dockingLight;
}

/*
* Function:
* uint8_t dfDockWithCamera(RobotGlobalStructure *sys)
*
* Function to guide the robot to the dock using the camera.
*
* Inputs:
* RobotGlobalStructure *sys
*   Pointer to the sys global data structure
*
* Returns:
* 0 when docking complete, otherwise non-zero
*
* Implementation:
* The docking function is a state machine that will change states after each step that is required
* for docking is performed. More to come [WIP]
*
* Improvements:
* [Ideas for improvements that are yet to be made](optional)
*
*/
uint8_t dfDockWithCamera(RobotGlobalStructure *sys)
{
	//static FDelayInstance delay;				//A delay instance for this function to use
	static float startFacing = 0;				//Used for rotating the robot relative to its
												//current facing
	static int8_t maxSection = 1;				//The section with the most pixels seen
	
	switch(sys->states.dockingCam)
	{
		case DCS_START:
			//If the camera isn't initialised, then we can do nothing.
			if(!sys->sensors.camera.initialised) break;
			//Code to power up the camera (Cos we don't want the camera running the whole time
			//Flattening our battery!)
			startFacing = sys->pos.facing;
			sys->states.dockingCam = DCS_SCAN_FOR_DOCK;
			break;
			
		case DCS_SCAN_FOR_DOCK:
		{
			uint16_t greenScores[DCS_SFD_SECTIONS];//Stores the dock position scores
			uint16_t maxVal = DCS_SFD_MIN_PIXELS;//The greatest number of pixels seen in a section

			//If a new frame has been written into the buffer and the robot isn't trying to turn
			if(!camBufferWriteFrame() 
			&& !mfRotateToHeading(startFacing + ((maxSection - (int)(DCS_SFD_SECTIONS/2))*10), sys))
			{
				//Scan a horizontal strip of the last frame for pixels that fall within the 
				//thresholds set in the constants above.
				sfCamScanForColour(DCS_SFD_START_LINE, DCS_SFD_END_LINE, dockingStationSig, 
									greenScores, DCS_SFD_SECTIONS);
				//See which section is the greatest:
				for(int i = 0; i < DCS_SFD_SECTIONS; i++)
				{
					if(greenScores[i] > maxVal)
					{
						maxVal = greenScores[i];
						maxSection = i;
					}
				}
				startFacing = sys->pos.facing;
				
				//If the dock appears in the centre of the camera view, start heading towards it.
				if(maxSection == (int)(DCS_SFD_SECTIONS/2))
					sys->states.dockingCam = DCS_DRIVE_TO_DOCK;
			}
		}
			break;
			
		case DCS_DRIVE_TO_DOCK:
			sys->states.dockingCam = DCS_ALIGN_DOCK;
			break;
			
		case DCS_ALIGN_DOCK:
			sys->states.dockingCam = DCS_COUPLE;
			break;
			
		case DCS_COUPLE:
			sys->states.dockingCam = DCS_FINISHED;
			break;
			
		case DCS_FINISHED:
			//Code to power down the camera
			maxSection = 1;
			sys->states.dockingCam = DCS_START;
			break;
	}
	
	return sys->states.dockingCam;
}

/*
* Function:
* uint8_t dfFollowLine(uint8_t speed, float *lineHeading, RobotGlobalStructure *sys)
*
* A basic function to follow a line
*
* Inputs:
* uint8_t speed:
*   Speed that robot will move at while following line (%)
* float *lineHeading:
*   Pointer to a float that will store the average heading that the line is believed to be on
* RobotGlobalStructure *sys
*   Pointer to the global robot data structure
*
* Returns:
* 0 when finished, otherwise current state
*
* Implementation:
* TODO:Update implementation description here -Matt
* Get the direction of the detected line. Multiply this by 15 and apply as a corrective heading
* to mfMoveToHeading.
*
* Improvements:
* Need to find a way to make it smoother.
*
*/
uint8_t dfFollowLine(uint8_t speed,	RobotGlobalStructure *sys)
{
	sys->flags.obaMoving = 1;
	
	switch(sys->states.followLine)
	{
		//Starting state. Has value of 0 so when line following is finished will return 0
		case FLS_START:
			sys->states.followLine = FLS_ALIGN;
			sys->sensors.line.pollInterval = 100;		//Poll line sensors every 100ms
			sys->sensors.prox.pollInterval = 100;
			sys->sensors.prox.pollEnabled = 0x01;		//Poll front sensor
			sys->sensors.colour.pollEnabled = 0;		//Don't poll colour
			sys->power.pollChargingStateEnabled = 0;	//Stop polling FC chip
			break;
		
		//Given the position of the line sensors relative to the wheels on the underside of the 
		//robot, the robot must stop and rotate on the spot in order to accurately locate the 
		//position of the line relative to the robot. The robot will rotate clockwise or anti-
		//clockwise depending on the directional data from the line sensors, and if the line is
		//detected as being directly underneath the robot, then that heading is recorded and the
		//function switches to the FOLLOW state.
		case FLS_ALIGN:
			if(sys->sensors.line.detected)
			{
				if(sys->sensors.line.direction < 0)
					{
						moveRobot(0,  -15 + sys->sensors.line.direction*5, 80);
					}
				if(sys->sensors.line.direction > 0)
				{
						moveRobot(0, 15 + sys->sensors.line.direction*5 , 80);
				}
				if(sys->sensors.line.direction == 0)
				{
					moveRobot(0, speed, 0);
				}
			} else {
				mfStopRobot(sys);
			}
			
			//If obstacle in front of us then stop.
			if(sys->sensors.prox.sensor[0] == PS_CLOSEST)
				sys->states.dockingLight = FLS_FINISH;
			
			break;
		
		//If finished, reset the state machine for next time and return a 0.
		case FLS_FINISH:
			sys->states.followLine = FLS_START;
			break;
	}
	return sys->states.followLine;	
}

/*
* Function:
* uint8_t dfScanBrightestLightSource(float *brightestHeading, uint16_t sweepAngle,
*									RobotGlobalStructure *sys);
*
* The robot will scan from -180 degrees to 180 degrees and record the heading with the brightest
* source of light (which hopefully is the charging station)
*
* Inputs:
* int16_t *brightestHeading
*   A pointer to a variable that contains a heading to the brightest detected light source so far.
* uint16_t sweepAngle:
*   The size of the arc to scan (360 would be a complete rotation)
* RobotGlobalStructure *sys
*   Pointer to the global robot data structure.
*
* Returns:
* Returns a 1 if the function hasn't completed yet, or a 0 if it has. When the function returns a 0
* it means the heading stored at *breightestHeading points to the brightest light source.
*
* Implementation:
* heading is a static variable that stores the heading that the robot is currently moving to.
* brightestVal is a static variable that stored the brightest detected light value so far.
* avgBrightness is a temporary variable that stores the average brightness between the two light
* sensors.
* First up the function calls the mfRotateToHeading function. If that function has completed (ie the
* robot is facing in the heading we want) then an average white light brightness reading is taken
* from the light sensors. If the average brightness just read is greater than the last stored
* brightness value then update brightestHeading with the current heading and update brightestVal
* with the current avgBrightness. Once the robot has rotated 360 degrees, return 0 and reset the
* static variable to their starting states to indicate that the scan is complete. The heading
* left behind in brightest heading is the heading with the greatest amount of light.
*
* Improvements:
* TODO: more comments
*
*/
uint8_t dfScanBrightestLightSource(float *brightestHeading, uint16_t sweepAngle, 
								 RobotGlobalStructure *sys)
{
	const float ROTATE_STEP_SZ = 3;
	static float startHeading;
	static float endHeading;
	static float sHeading;
	static uint32_t brightestVal;
	float rotateError;
	uint32_t avgBrightness = 0;
	uint32_t avgBrightnessOld = 0xFFFF;	//Only looks for positive delta, so by making the first
										//value high, the first delta reading will be negative,
										//preventing a false positive
	
	switch(sys->states.scanBrightest)
	{
		case SBS_FUNCTION_INIT:
			//Calculate where to start sweep from
			brightestVal = 0;								//Reset brightestValue
			startHeading = sys->pos.facing - (sweepAngle/2);//Calculate start heading
			endHeading = startHeading + sweepAngle;
			sHeading = startHeading + sweepAngle/3;
			sys->states.scanBrightest = SBS_GOTO_START_POSITION;	//Angles set up, lets start
			return 1;
			break;

		case SBS_GOTO_START_POSITION:
			if(!mfRotateToHeading(startHeading, sys))
				sys->states.scanBrightest = SBS_SWEEP;			//In position, now perform sweep
			return 1;
			break;
		
		case SBS_SWEEP:
			rotateError = mfRotateToHeading(sHeading, sys);
			if(abs(rotateError) < 170 && sHeading < endHeading)//Keep sHeading only 25 degrees ahead
															//of current heading so that robot will
															//always take the long way around
			{
				sHeading += ROTATE_STEP_SZ;
				if(sHeading > endHeading)
					sHeading = endHeading;
			}
			if(!rotateError)
				sys->states.scanBrightest = SBS_END;
			else
			{
				avgBrightness = (sys->sensors.colour.left.green + sys->sensors.colour.right.green)/2;
				if((avgBrightness - avgBrightnessOld) > brightestVal)
				{
					brightestVal = avgBrightness - avgBrightnessOld;
					*brightestHeading = sys->pos.facing;
				}
				avgBrightnessOld = avgBrightness;
			}
			return 1;
			break;
		
		case SBS_END:
			sys->states.scanBrightest = SBS_FUNCTION_INIT;
			return 0;
	}
	return 1;
}

/*
* Function:
* float dfScanBrightestLightSourceProx(void)
*
* Uses all of the proximity sensors simultaneously to find the brightest source of light.
*
* Inputs:
* none
*
* Returns:
* Heading angle at which the brightest light source was detected.
*
* Implementation:
* The sensor array holds the values retrieved from each sensor. brightestVal holds the brightest
* detected value from any of the sensors. brightest sensor holds the index number of the sensor
* with the brightest ambient light detected.
* First the function enables ambient light detection on the proximity sensors. Then it reads the 
* light reading from each one into the sensor array. After this, proximity mode is re enabled on the
* proximity sensors. After that a for loop is used to see which sensor contained the brightes value.
* Finally, the number of the sensor multiplied by 60 (the angle in degrees that each sensor is
* apart) is returned from the function, indicating the direction of the brightest light source.
*
* Improvements:
* [NOT WORKING]: When proxAmbModeEnabled() is called, the IMU stops updating. I think its todo with
* the delay function that waits 50ms for data to be ready. Need to do more experimentation. -Matt
*
*/
float dfScanBrightestLightSourceProx(void)
{
	uint16_t sensor[6];
	uint16_t brightestVal = 0;
	int brightestSensor = 0;
	//Enable Ambient light mode on the prox sensors
	proxAmbModeEnabled();

	//Read light sensor values
	sensor[0] = proxAmbRead(MUX_PROXSENS_A);		//0
	sensor[1] = proxAmbRead(MUX_PROXSENS_B);		//60
	sensor[2] = proxAmbRead(MUX_PROXSENS_C);		//120
	sensor[3] = proxAmbRead(MUX_PROXSENS_D);		//180
	sensor[4] = proxAmbRead(MUX_PROXSENS_E);		//-120
	sensor[5] = proxAmbRead(MUX_PROXSENS_F);		//-60
	//Revert to proximity mode
	proxModeEnabled();
	
	//Find largest
	for (int i = 0; i < 6; i++)
	{
		if(sensor[i] > brightestVal)
		{
			brightestVal = sensor[i];
			brightestSensor = i;
		}
	}
	
	return nfWrapAngle(60.0*brightestSensor);
}