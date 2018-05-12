 /*
* motion_functions.h
*
* Author : Matthew Witt (pxf5695@autuni.ac.nz)
* Created: 13/08/2017 12:42:12 AM
*
* Project Repository: https://github.com/wittsend/aut-swarm-robotics
*
* Defines and function prototypes for motion_function.c which provides high level movement functions
*
* More Info:
* Atmel SAM 4N Processor Datasheet:http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
* Relevant reference materials or datasheets if applicable
*
* Functions:
* void mfExecuteMotionInstruction(RobotGlobalStructure *sys);
* void mfStopRobot(RobotGlobalStructure *sys);
* float mfRotateToHeading(float heading, float speed, RobotGlobalStructure *sys);
* float mfMoveToHeading(float heading, uint8_t speed, RobotGlobalStructure *sys);
* float mfMoveToHeadingByDistance(float heading, uint8_t speed, float distance,
*	RobotGlobalStructure *sys);
* float mfTrackLight(uint8_t speed, RobotGlobalStructure *sys);
* float mfTrackProx(uint8_t speed, RobotGlobalStructure *sys);
* char mfRandomMovementGenerator(RobotGlobalStructure *sys);
* float mfAdvancedMove(float heading, float facing, uint8_t speed,
*	uint8_t maxTurnRatio, RobotGlobalStructure *sys);
* int32_t mfMoveToPosition(int32_t x, int32_t y, uint8_t speed, float facing,
*	uint8_t maxTurnRatio, RobotGlobalStructure *sys);
*
*/

#ifndef MOTION_FUNCTIONS_H_
#define MOTION_FUNCTIONS_H_

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include <stdlib.h>

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
/*
* Function:
* void mfExecuteMotionInstruction(RobotGlobalStructure *sys)
*
* Will execute the motion instruction stored in the move sub structure in sys. This is now the sole
* interface of the motion_functions module.
*
* RobotGlobalStructure *sys:
*   A pointer to the sys global data structure
*
* Returns:
* none
*
*/
void mfExecuteMotionInstruction(RobotGlobalStructure *sys);

/*
* Function:
* void mfStopRobot(RobotGlobalStructure *sys)
*
* Stops the robot (High level function to avoid direct access to the motor_driver from other
* high level functions
*
* Inputs:
* RobotGlobalStructure *sys
*   Pointer to the global robot data structure
*
* Returns:
* none
*
*/
void mfStopRobot(RobotGlobalStructure *sys);

/*
* Function:
* float mfRotateToHeading(float heading, int8_t maxSpeed, RobotGlobalStructure *sys)
*
* Will rotate the robot to face the given heading
*
* Inputs:
* float heading:
*   The heading in degrees that we wish the robot to face (-180 < heading < 180)
* RobotGlobalStructure *sys:
*   A pointer to the sys->pos. structure so we can get IMU.yaw
*
* Returns:
* Will return 0 if the robot has settled at the desired heading, otherwise will return the signed
* error
*
*/
float mfRotateToHeading(float heading, float speed, RobotGlobalStructure *sys);

/*
* Function:
* float mfMoveToHeading(float heading, uint8_t speed, RobotGlobalStructure *sys)
*
* Will rotate and then move the robot along the given heading at the given speed.
*
* Inputs:
* float heading:
*   The heading in degrees that we wish the robot to face (-180 < heading < 180)
* uint8_t speed:
*   Absolute speed as a percentage of maximum speed (0-100)
* RobotGlobalStructure *sys:
*   A pointer to the sys->pos. structure so we can get IMU.yaw
*
* Returns:
* Will return 0 if the robot moving along the desired heading, otherwise will return the signed
* error
*
*/
float mfMoveToHeading(float heading, uint8_t speed, RobotGlobalStructure *sys);

/*
* Function:
* float mfMoveToHeadingByDistance(float heading, uint8_t speed, float distance,
*									 RobotGlobalStructure *sys);
*
* Will allow robot to move along the given heading a given distance.
*
* Inputs:
* float heading:
*   Heading to move along (-180 to 180 degrees)
* uint8_t speed:
*   Percentage of max speed to move at (0-100%)
* float distance:
*   Distance to travel before stopping.
* struct SystemStatesGroup *state
*   Pointer to the sys.states data structure
* RobotGlobalStructure *sys:
*   Pointer to the sys->pos. global structure.
*
* Returns:
* 0 when maneuver is complete, otherwise returns distance remaining before maneuver complete.
*
*/
float mfMoveToHeadingByDistance(float heading, uint8_t speed, float distance,
RobotGlobalStructure *sys);

/*
* Function:
* float mfTrackLight(RobotGlobalStructure *sys)
*
* Robot will attempt to aim itself at a light source using colour sensors
*
* Inputs:
* RobotGlobalStructure *sys:
*   A pointer to the sys->pos. structure
*
* Returns:
* 0 if equilibrium is reached, otherwise will return the proportional error value
*
*/
float mfTrackLight(uint8_t speed, RobotGlobalStructure *sys);

/*
* Function:
* float mfTrackLightProx(RobotGlobalStructure *sys)
*
* Function to track a light source using the proximity sensors. [WIP]
*
* Inputs:
* RobotGlobalStructure *sys:
*   Pointer to the global sys->pos. data structure.
*
* Returns:
* 0 if facing light source, otherwise will return heading error value
*
*/
float mfTrackProx(uint8_t speed, RobotGlobalStructure *sys);

/*
* Function:
* char mfRandomMovementGenerator(void)
*
* Will make the robot move around psuedo-randomly
*
* Inputs:
* No Inputs
*
* Returns:
* No return values
*
*/
char mfRandomMovementGenerator(RobotGlobalStructure *sys);

/*
* Function:
* char mfAdvancedMove(float heading, float facing, uint8_t speed,
* 							uint8_t maxTurnRatio, RobotGlobalStructure *sys)
*
* Will move the robot along a heading, and also rotate the robot to face the given facing,
* using closed loop control from the mouse and IMU to achieve it.
*
* Inputs:
* float heading:
*   The absolute (arena) heading that the robot should travel along
* float facing:
*   The absolute (arena) facing that the robot should face
* uint8_t speed:
*   The maximum motor speed (0-100%)
* uint8_t maxTurnRatio:
*   The percentage of rotational speed to be applied to the motors (ie how fast the robot will
*   rotate towards the desired facing). 0% means the robot will not rotate at all. %100 means that
*   The robot will only rotate on the spot until the desired facing is achieve before setting off
*   on the desired heading. Anything in between will have the robot gradually rotate while
*   travelling along the desired heading.
* RobotGlobalStructure *sys:
*   Pointer to the global robot data structure
*
* Returns:
* 0 when the robot has achieved the desired facing, otherwise the proportional error between the
* desired facing and the current facing of the robot.
*
*/
float mfAdvancedMove(float heading, float facing, uint8_t speed,
uint8_t maxTurnRatio, RobotGlobalStructure *sys);

/*
* Function:
* int32_t mfMoveToPosition(int32_t x, int32_t y, uint8_t speed, float facing,
*							uint8_t maxTurnRatio, RobotGlobalStructure *sys)
*
* Will move the robot to the given position in the arena
*
* Inputs:
* int32_t x:
*   The absolute (arena) x position (mm) that the robot should drive to
* int32_t y:
*   The absolute (arena) y position (mm) that the robot should drive to
* uint8_t speed:
*   The maximum motor speed (0-100%)
* float facing:
*   The absolute (arena) facing that the robot should face
* uint8_t maxTurnRatio:
*   The percentage of rotational speed to be applied to the motors (ie how fast the robot will
*   rotate towards the desired facing). 0% means the robot will not rotate at all. %100 means that
*   The robot will only rotate on the spot until the desired facing is achieve before setting off
*   on the desired heading. Anything in between will have the robot gradually rotate while
*   travelling along the desired heading.
* RobotGlobalStructure *sys:
*   Pointer to the global robot data structure
*
* Returns:
* current state
*
*/
int32_t mfMoveToPosition(int32_t x, int32_t y, uint8_t speed, float facing,
uint8_t maxTurnRatio, RobotGlobalStructure *sys);

#endif /* MOTION_FUNCTIONS_H_ */