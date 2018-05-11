/*
* docking_functions.h
*
* Author : Matthew Witt (pxf5695@autuni.ac.nz)
* Created: 25/07/2017 8:08:11 PM
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
* uint8_t dfDockWithCamera(RobotGlobalStructure *sys)
* uint8_t dfFollowLine(uint8_t speed, float *lineHeading, RobotGlobalStructure *sys)
* uint8_t dfScanBrightestLightSource(int16_t *brightestHeading)
* float dfScanBrightestLightSourceProx(void)
*
*/

#ifndef DOCKING_FUNCTIONS_H_
#define DOCKING_FUNCTIONS_H_

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
/*
* Function:
* void dfDockWithLightSensor(void)
*
* Function to guide the robot to the dock.
*
* Inputs:
* none
*
* Returns:
* none
*
*/
uint8_t dfDockWithLightSensor(RobotGlobalStructure *sys);

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
*/
uint8_t dfDockWithCamera(RobotGlobalStructure *sys);

/*
* Function:
* uint8_t dfFollowLine(uint8_t speed, float *lineHeading, RobotGlobalStructure *sys)
*
* A basic function to follow a line that seems to work ok
*
* Inputs:
* uint8_t speed:
*   Speed that robot will move at while following line (%)
* float *lineHeading:
*   Pointer to a float that will store the average heading that the line is believed to be on
* struct Poistion *sys->pos
*   Pointer to the sys->pos. data structure
*
* Returns:
* 0 when finished, otherwise current state
*
*/
uint8_t dfFollowLine(uint8_t speed, RobotGlobalStructure *sys);

/*
* Function:
* uint8_t dfScanBrightestLightSource(float *brightestHeading, uint16_t sweepAngle, RobotGlobalStructure *sys)
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
*/
uint8_t dfScanBrightestLightSource(float *brightestHeading, uint16_t sweepAngle, RobotGlobalStructure *sys);

/*
* Function:
* float dfScanBrightestLightSourceProx(void)
*
* Uses all of the proximity sensors simultaneously to find the brightest source of light.
*
* Inputs:
* float *brightestHeading
* Pointer to a float where the brightest heading will be stored once it has been retrieved.
*
* Returns:
* 0 when the brightest heading has been determined
*
*/
uint8_t dfScanBrightestLightSourceProx(float *brightestHeading, RobotGlobalStructure *sys);

#endif /* DOCKING_FUNCTIONS_H_ */