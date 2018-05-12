/*
* robot_setup.h
*
* Author : Adam Parlane, Matthew Witt
* Created: 6/7/2017
*
* Project Repository: https://github.com/wittsend/aut-swarm-robotics
*
* Contains the misc defines that havnt been modularised as of yet (6/7)
* Also has all the headers so it can just be included in every header giving access to everything.
* There are compiler directives that will compile defines specific to each version of the robot.
*
* More Info:
* Atmel SAM 4N Processor Datasheet:http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
*
* Functions:
* void masterClockInit(void)
* uint8_t waitForFlag(const volatile uint32_t *regAddr, uint32_t regMask, uint16_t timeOutMs)
* int32_t capToRangeInt(int32_t valueToCap, int32_t minimumVal, int32_t maximumVal)
* uint32_t capToRangeUint(uint32_t valueToCap, uint32_t minimumVal, uint32_t maximumVal)
* float capToRangeFlt(float valueToCap, float minimumVal, float maximumVal)
*
*/

#ifndef ROBOTDEFINES_H_
#define ROBOTDEFINES_H_

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
//These are includes that we want everywhere in the code (as robot setup is included everywhere)
#include "Interfaces/spi.h"		//Fixes SPI issue
#include "sam.h"				//Micro controller specific defines
#include <stdint.h>				//Gives standard integer type definitions (ie uint8_t)
#include <stdbool.h>			//Gives boolean variable types
#include "Interfaces/xbee_driver.h"//Gives access to MessageInfo structure definition
#include "Interfaces/prox_sens_interface.h"//Gives access to ProximityMode enum

//////////////[Enumerations]////////////////////////////////////////////////////////////////////////
//The following enumerations represent states in each state machine in the system
//Don't add new ones or change the order without also updating the PC GUI
typedef enum MainStates
//main() function states
{
	M_TEST,
	M_TEST_ALL,
	M_MANUAL,
	M_FORMATION,
	M_DOCKING,
	M_OBSTACLE_AVOIDANCE,
	M_OBSTACLE_AVOIDANCE_DEMO,
	M_IDLE, 
	M_CHARGING,
	M_LINE_FOLLOW,
	M_LIGHT_FOLLOW, 
	M_RANDOM,
	M_MOVE_TO_POSITION,
	M_ROTATE_TO_FACING,
	M_STARTUP_DELAY,
	M_IMU_CALIBRATION,
	M_DOCKING_OLD,
	M_INITIALISATION
} MainStates;

typedef enum DockingStates
//dfDockWithLightSensor() function states
{
	DS_FINISHED,
	DS_START,
	DS_FACE_BRIGHTEST,
	DS_MOVE_FORWARD,
	DS_RESCAN_BRIGHTEST,
	DS_FOLLOW_LINE,
	DS_CHRG_CONNECT,
	DS_CHRG_NOT_FOUND
} DockingStates;

typedef enum DockingWithCamStates
{
	DCS_FINISHED,
	DCS_START,
	DCS_SCAN_FOR_DOCK,
	DCS_FACE_DOCK,
	DCS_DRIVE_TO_DOCK,
	DCS_ALIGN_DOCK,
	DCS_COUPLE
} DockingWithCamStates;

typedef enum FollowLineStates
//dfFollowLine() function states
{
	FLS_START,
	FLS_ALIGN,
	FLS_FINISH
} FollowLineStates;

typedef enum ScanBrightestStates
//mfScanBrightestLightSource() function states
{
	SBS_FUNCTION_INIT,
	SBS_GOTO_START_POSITION,
	SBS_SWEEP,
	SBS_END
} ScanBrightestStates;

typedef enum ChargeCycleStates
//pfChargeCycleHandler() function states
{
	CCS_FINISHED,
	CCS_CHECK_POWER,
	CCS_CHARGING,
	CCS_RECONNECT,
	CCS_FAULT,
	CCS_DISMOUNT,
	CCS_TURN_AWAY,
	CCS_STOP_POLLING
} ChargeCycleStates;

typedef enum MTHByDistanceStates
//mfMoveToHeadingByDistance() function states
{
	MHD_START,
	MHD_MOVING,
	MHD_STOP
} MTHByDistanceStates;

//Movement Instructions
typedef enum MovementInstructionSet
{
	MI_STOP,
	MI_ROTATE_TO_HEADING,
	MI_MOVE_TO_HEADING,
	MI_MOVE_TO_HEADING_BY_DIST,
	MI_ADVANCED_MOVE,
	MI_MOVE_TO_POSITION,
	MI_TRACK_LIGHT,
	MI_TRACK_LIGHT_PROX,
	MI_RANDOM_MOVE
} MovementInstructionSet;

////////////////[Type Definitions]//////////////////////////////////////////////////////////////////
//Stores optical sensor raw and derived data
typedef struct OpticalSensor
{
	int dx;				//Rate of change from optical sensor (X axis is left to right)
	int dy;				//Rate of change from optical sensor (Y axis is fwd/bckwd)
	int dxSum;			//Sum of samples between updates from optical sensor (for rolling average)
	int dySum;			//Sum of samples between updates from optical sensor (for rolling average)
	uint16_t sampleCount;	//Number of samples between updates (for rolling average)
	int x;				//Count sum on x axis
	int y;				//Count sum on y axis
	int xOld;			//x value at the last PC update
	int yOld;			//y value at the last PC update
	char pollEnabled;	//Enable polling the optical sensor
	char pollInterval;	//Rate at which to poll Mouse
	char overflowFlag;	//1 if data has overflowed on optical sensor
	uint8_t surfaceQuality;	//A value signifying quality of the surface (242 = max quality)
	float convFactor;	//A coefficient to convert to mm
} OpticalSensor;

//Stores IMU sensor raw and converted data
typedef struct IMUSensor
{
	long qw;			//W component of the quaternion complex number returned by DMP
	long qx;			//X component of the quaternion complex number returned by DMP
	long qy;			//Y component of the quaternion complex number returned by DMP
	long qz;			//Z component of the quaternion complex number returned by DMP
	float accelX;		//Delta X Acceleration in ms^2
	float accelY;		//Delta Y Acceleration in ms^2
	float accelZ;		//Delta Z Acceleration in ms^2
	float accelXBias;	//Used to "level" the acceleromter. Calculated by nfCalcAcceleromterBias()
	float accelYBias;
	float gyroX;		//Delta pitch in deg/s
	float gyroY;		//Delta roll in	deg/s
	float gyroZ;		//Delta yaw in deg/s (Delta heading)
	float pitch;		//Absolute pitch from DMP (degrees)
	float roll;			//Absolute roll from DMP (degrees)
	float yaw;			//Absolute yaw (heading) from DMP (degrees)
	float gMag;			//Magnitude of the gravitational vector
	char dmpEnabled;	//A flag that states whether or not the DMP is enabled
	char pollEnabled;	//Enable polling the IMU
	uint16_t pollRate;	//Rate at which the IMU will send new data to uC (Hz)
	bool gyroCalEnabled;	//Enable gyro calibration on startup.
} IMUSensor;

//Stores the current closed loop movement instruction. Movement instructions and parameters are
//passed to this structure instead of directly calling the motion functions. This means that the
//Motion functions will always be updated from the timer interrupt polling.
typedef struct MovementInstruction
{
	MovementInstructionSet cmd;
	float heading;			//RTH, MTH, MTHBD, AdvMove
	float speed;			//RTH, MTH, MTHBD, AdvMove, MoveToPos (Should be all really)
	float dist;				//Move to heading by dist
	float facing;			//Move to position, Advanced Move
	uint8_t maxTurnRatio;	//Move to position, Advanced Move
	int32_t x;				//Move to position
	int32_t y;				//Move to position
	float status;			//Will usually be proportional error (how far off target the robot is)
	bool executed;			//Whether this command has been executed yet or not.
} MovementInstruction; 

//structure to store all the robot side navigation / positioning data
//this will be written to by getMouseXY, nfGetEulerAngles, and another navigation function which
//combines them. The structure will store the relevant info from both key sensors and fuse them in
//an additional function (84bytes i think)
typedef struct PositionGroup
{
	OpticalSensor Optical;		//Optical sensor raw data
	IMUSensor IMU;				//IMU raw and converted data
	float x;					//Absolute X position in arena (mm)
	float y;					//Absolute Y position in arena (mm)
	float dx;					//delta x in mm
	float dy;					//delta y in mm
	float speed;				//Speed in mm per second
	float heading;				//Absolute direction of travel (deg)
	float relHeading;			//Relative heading of travel (to front of robot)
	float facing;				//Absolute direction robot is facing (deg)
	signed int targetHeading;	//For obstacle avoidance, desired heading before an obstacle is 
								//detected
	char targetSpeed;			//For obstacle avoidance, desired speed
	unsigned long timeStamp;	//Time at which last IMU reading took place (ms). Can be used as a
								//time marker for all Nav data, as it all get polled at the same
								//time as the IMU
	float deltaTime;			//Time between last IMU reading and IMU previous reading
	float facingOffset;			//Used to offset facing value (when corrected by PC)
	int32_t oldPCX;				//The last X position from the PC
	int32_t oldPCY;				//The last Y position from the PC
	int32_t targetX;			//The target x position in mm from the PC
	int32_t targetY;			//The target y position in mm from the PC
} PositionGroup;

//Stores information about the battery and charging
typedef struct BatteryChargeData
{
	uint16_t batteryVoltage;			//Battery voltage in mV
	uint8_t batteryTemp;				//Battery temperature in degrees
	uint16_t batteryMaxVoltage;			//Fully charged voltage of battery (will calibrate onthefly)
	uint16_t batteryDockingVoltage;		//Voltage below which the robot should seek dock
	uint16_t batteryMinVoltage;			//Voltage at which robot is considered completely dead
	uint8_t batteryPercentage;			//Percentage of battery remaining
	uint8_t fcChipStatus;				//Status or fault code reported by Charge chip
	uint8_t fcChipFaultFlag;			//Fault detected by charge chip, see Status for code
	uint8_t pollBatteryEnabled;			//Enable battery voltage polling
	uint16_t pollBatteryInterval;		//Interval in ms to poll battery voltage/temp
	uint8_t pollChargingStateEnabled;	//Enable charge chip status polling
	uint16_t pollChargingStateInterval;	//Interval in ms to poll charge chip
	uint8_t chargeWatchDogEnabled;		//Enable the FC chip watchdog system (for when charging)
	uint16_t chargeWatchDogInterval;	//Watchdog routine interval (ms)
} BatteryChargeData;

//Stores colour sensor data, both raw and converted, for a single colour sensor
typedef struct ColourSensorData
{
	uint16_t red;
	uint16_t green;
	uint16_t blue;
	uint16_t white;
	uint16_t hue;
	uint16_t saturation;
	uint16_t value;
} ColourSensorData;

//Will store states of the line sensors. This is necessary
//because there is a gray area when the sensor is half on and half off the line, so by establishing
//hysteresis and only changing the stored states when an upper and lower threshold is crossed,
//jitter should be reduced.
typedef struct LineSensorArray
{
	uint8_t outerLeft;
	uint8_t innerLeft;
	uint8_t innerRight;
	uint8_t outerRight;
	uint16_t pollInterval;
	uint8_t pollEnabled;
	int8_t direction;
	uint8_t detected;
} LineSensorArray;

struct transmitDataStructure
{
	char Data[50];//array for data to be transmitted to PC BEFORE XBee framing has been added
	uint8_t DataSize;//size of the transmit array
};

typedef struct CommunicationDataGroup
{
	bool twi2SlavePollEnabled;			// Flag to enable or disable checking of slave requests on twi2 (from top mounted LCD)
	uint16_t twi2SlavePollInterval;	// Interval at which to poll at (ms)
	uint8_t twi2ReceivedDataByte;		// Stores the last received data byte from TWI2 slave

	bool pcUpdateEnable;				// Flag to enable or disable PC update/status messages
	uint16_t pcUpdateInterval;			// Interval at which the PC is updated (ms)
	
	bool xbeeNewData;					// Flag to indicate new xbee data has been received
	uint8_t xbeeMessageType;			// The type of the received xbee message
	uint8_t xbeeMessageData [100];		// Buffer for received xbee messages
	uint16_t testModeStreamInterval;	// Interval between streaming test data packets back to the PC (ms)
	struct transmitDataStructure transmitData;	// ?? transmit buffer ??
	uint64_t xbeeMissedMessages;		// Number of xbee messages that have been missed due to not been check fast enough
} CommunicationDataGroup;

//Proximity sensor sub-structure
typedef struct ProximitySensorGroup
{
	uint16_t sensor[6];
	uint8_t pollEnabled;		//Bitmask of the sensors being polled
	uint16_t pollInterval;
	uint8_t errorCount;
	ProximityMode status;			//Indicates if the prox sensors are sensing ambient light
	ProximityMode setMode;			//Is written to to set the desired mode of the prox sensors
}ProximitySensorGroup;

//Structure that will store all system flags for global use
typedef struct SystemFlagsGroup
{
	char imuCheckFifo;	//IMU ext interrupt has been triggered
	char camBufferRead;	//A new image is ready to be read from the camera FIFO buffer
	char twi2NewData;	//New data received on twi2 (Slave interface)
	char obaMoving;		//Robot is in motion
	char obaEnabled;	//Obstacle avoidance enabled
	char cornerFlag;
	char posPCNewData;	//New position information from the PC
} SystemFlagsGroup;

//Structure that will store the state of every state machine in the system
typedef struct SystemStatesGroup
{
	//Main function state machine state
	MainStates mainf;
	MainStates mainfPrev;
	
	//dfDockWithLightSensor() states
	DockingStates dockingLight;
	DockingWithCamStates dockingCam;
	
	//dfFollowLine() states
	FollowLineStates followLine;
	
	//dfScanBrightestLightSource() states
	ScanBrightestStates scanBrightest;
	
	//pfChargeCycleHandler() states
	ChargeCycleStates chargeCycle;
	
	//mfMoveToHeadingByDistance() states
	MTHByDistanceStates moveHeadingDistance;
} SystemStatesGroup;

//Colour sensor sub structure
typedef struct ColourSensorGroup
{
	ColourSensorData left;
	ColourSensorData right;
	uint16_t pollInterval;		//Poll rate
	uint8_t pollEnabled;		//Contains a bitmask indicating which sensors are updated
	uint8_t getHSV;				//Whether or not to convert to HSV when retrieving
} ColourSensorGroup;

//Camera sensor sub structure
typedef struct CameraSensorGroup
{
	bool initialised;	//flag to indicate if the camera has been successfully initialised
} CameraSensorGroup;

//Sensors sub-structure
typedef struct SensorDataGroup
{
	LineSensorArray line;
	ColourSensorGroup colour;
	ProximitySensorGroup prox;
	CameraSensorGroup camera;
} SensorDataGroup;

//Root Structure to combine all system globals
typedef struct RobotGlobalStructure
{
	SystemStatesGroup states;				//System states
	SystemFlagsGroup flags;					//System global flags
	SensorDataGroup sensors;				//Sensor data
	CommunicationDataGroup comms;			//Communication system control and data
	MovementInstruction move;				//Supply command here to move robot.
	PositionGroup pos;						//Position information
	BatteryChargeData power;				//Battery/Charging info and control
	uint32_t timeStamp;						//System timestamp (millisecs since power on)
	uint16_t startupDelay;					//Time to wait between sys setup and execution
	uint16_t sysTaskInterval;				//Delay between system task executions.
	bool debugStrings;						//Whether debug strings are enabled or not
} RobotGlobalStructure;

//////////////[Defines]/////////////////////////////////////////////////////////////////////////////
//Fixes an issue in Atmel's CMSIS implementation
#define REG_PIOA_ABCDSR1 (*(__IO uint32_t*)0x400E0E70U)
#define REG_PIOA_ABCDSR2 (*(__IO uint32_t*)0x400E0E74U)

//////////////[Global variables]////////////////////////////////////////////////////////////////////
//Global variables should be initialised in robot_setup.c, then an extern to them should be placed
//here, otherwise we end up with multiple definition errors.
extern RobotGlobalStructure sys;

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
/*
* Function:
* void robotSetup(void)
*
* The initialisation routine for all hardware in the robot.
*
* Inputs:
* none
*
* Returns:
* none
*
*/
void robotSetup(void);

/*
* Function:
* void performSystemTasks(RobotGlobalStructure *sys)
*
* Executes all system critical tasks. These function used to get called at the bottom of the main
* loop, but now they get called regularly by a timer interrupt at the interval set by 
* sys.sysTaskInterval. This ensures that PID control and communication happens when it should.
*
* Inputs:
* RobotGlobalStructure *sys:
*   A pointer to the sys global data structure
*
* Returns:
* none
*
* Implementation:
* Executes all the polling functions. This should ONLY be executed by timer interrupt.
*
*/
void performSystemTasks(RobotGlobalStructure *sys);

/*
* Function:
* uint8_t waitForFlag(uint32_t *regAddr, uint32_t regMask, uint16_t timeOutMs)
*
* Will wait for the given status bit to become true. If it doesn't become true in the time
* specified in timeOutMs, then the function exits with an error.
*
* Inputs:
* uint32_t *regAddr
*	The address to the status register that is to be monitored.
* uint32_t regMask
*   The bit mask to apply to the given register.
* uint16_t timeOutMs
*   The maximum number of milliseconds to wait before exiting the function with an error.
*
* Returns:
* 0 if flag was detected or 1 if timeout was reached before flag was detected.
*
*/
uint8_t waitForFlag(const volatile uint32_t *regAddr, uint32_t regMask, uint16_t timeOutMs);

/*
* Function:
* type capToRangeInt(type valueToCap, type minimumVal, type maximumVal)
*
* Will see if a value is within a given range. If it is outside the given range, then limit the
* value to the given minimum or maximum value. Three different versions of this function operate on
* different types of variable. (Signed and unsigned integers, and single precision floating point
* numbers.
*
* Inputs:
* valueToCap:
*   The number we are checking to see if it is in range.
* minimumVal:
*   The minimumValue that we would like valueToCap to be
* maximumVal:
*   The maximum value we would like valueToCap to be.
*
* Returns:
* If valueToCap was outside the desired range, then a range limited version of valueToCap is
* returned, otherwise valueToCap is returned unmodified.
*
*/
int32_t capToRangeInt(int32_t valueToCap, int32_t minimumVal, int32_t maximumVal);

uint32_t capToRangeUint(uint32_t valueToCap, uint32_t minimumVal, uint32_t maximumVal);

float capToRangeFlt(float valueToCap, float minimumVal, float maximumVal);

//Convert double to string
char * dtoa(char *s, double n);

#endif /* ROBOTDEFINES_H_ */