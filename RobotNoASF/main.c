/*
* main.c
*
* Author : et. al
* Created: Unknown
*
* Project Repository: https://github.com/AdamParlane/aut-swarm-robotics
*
* main c file for robot source code for BE (Hons) / BEng Tech Final year industrial project 2017
*
* More Info:
* Atmel SAM 4N Processor Datasheet:http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
* Relevant reference materials or datasheets if applicable
*
* Functions:
* int main(void)
* 
*/

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "robot_setup.h"

#include "Interfaces/xbee_driver.h"
#include "Interfaces/adc_interface.h"
#include "Interfaces/pio_interface.h"
#include "Interfaces/timer_interface.h"

#include "Functions/charging_functions.h"
#include "Functions/docking_functions.h"
#include "Functions/manual_mode.h"
#include "Functions/motion_functions.h"
#include "Functions/navigation_functions.h"
#include "Functions/obstacle_avoidance.h"
#include "Functions/testFunctions.h"

//////////////[Defines]/////////////////////////////////////////////////////////////////////////////

//////////////[Global variables]////////////////////////////////////////////////////////////////////
uint16_t battVoltage;					//Stores battery voltage on start up
extern RobotGlobalStructure sys;		//System data structure
extern struct MessageInfo message;		//Incoming message structure

//////////////[Functions]///////////////////////////////////////////////////////////////////////////

/*
* Function:
* int main(void)
*
* Overview of robot code execution
*
* Inputs:
* none
*
* Returns:
* N/A
*
* Implementation:
* 
* The main system runs on a state machine in a while(1) loop to control the robot
* Before this state machine is run robotSetup() is called to initialise all peripherals
* The main robot state is initialised at IDLE to allow the user (on PC GUI) to input control
*
* The cases are:
* TEST:			***module: test_functions.c/h
*				System test where the robot sends back peripheral data when requested from the PC
*				This may be 1 sample of data or streaming data every 100ms
*				Used to ensure the robot is functioning correctly and the data can be read from the GUI
* MANUAL:		***module: manual_mode.c/h
*				Manual mode allows the user to drive the robot around in real time
*				Again this is called and controlled by the GUI
*				Movement options are move (N, NE, E, SE, S, SW, W, NW) rotate (CW, CCW) at any speed
*				The manualControl function which handles the command is called when it get a new command
*				Charge detector is called to check if the user has docked the robot
* DOCKING:		***module: docking_functions.c/h
*				Executes the ideal docking routine to charge the robot
*				This function returns 0 when complete and changes states to IDLE
* LINE_FOLLOW:	***module: motion_functions.c/h
*				Executes the line follow routine to either charge the robot or just follow a line
*				This function returns 0 when complete and changes states to IDLE
* LIGHT_FOLLOW:	***module: motion_functions.c/h
*				Executes the light follow routine to either charge the robot or just follow a light
* FORMATION:	***module: [WIP]
* CHARGING:		***module: fc_Interface.c/h
*				Checks the robot is still charging and the status of the battery
*				Entered when the robot is docked
*				Charge_info also contains the charging status should other functions require it
* IDLE:			***module: N/A
*				Stops the robot motion by calling mfStopRobot()
*				Blinks LED 3 every 500ms to show externally that robot is in IDLE
*				Entered when other majour blocking functions or states are finished
*				Exitied by PC commands
*
* After the state machine 3 functions are run every loop to check key peripherals
* These are:	xbeeGetNew()			checks and handles incoming commands
*				nfRetrieveData()				updates the robot's navigation structure
*				dodgeObstacle()					executes obstacle avoidance routine
* Note that doegeObstacle() is guarded by 2 flags: obstacleAvoidance enabled and the robot is moving
*
* Improvements:
*
* More functionality incoming (formations, smarter obstacle avoidance, high level error codes etc)
*
*/
int main(void)
{
	robotSetup(); //Set up the system and peripherals
	battVoltage = adcBatteryVoltage();	//Add to your watch to keep an eye on the battery
	sys.states.mainf = M_IDLE; //start system at IDLE
	sys.states.mainfPrev = M_IDLE;
	uint8_t chargeCycleReturn = 0;
	uint8_t dockingReturn = 0;
	float lineHeading = 0;

	while(1)
	{
		switch (sys.states.mainf)
		{
			case M_TEST: //System Test Mode
			//Entered when test command received from PC
				testManager(message, &sys); //Interprets test command and executes it
				break;
			
			case M_MANUAL: //User controlled mode
			//Entered when manual movement command received from PC
				if(sys.flags.xbeeNewData) //if there is new data
					manualControl(message, &sys);
				break;
			
			case M_DOCKING:
			//if battery low or manual docking command sent from PC
				dockingReturn = dfDockRobot(&sys);
				if(!dockingReturn)	//Execute docking procedure state machine
					sys.states.mainf = M_CHARGING;		//If finished docking, switch to charging
				else if(dockingReturn == 7)
					sys.states.mainf = M_IDLE;			//If charger connection failed
				break;
			
			case M_LINE_FOLLOW:
			//Entered when line follow command received from PC
				if(!dfFollowLine(35, &lineHeading, &sys))//Line follower will return 0 when complete
					sys.states.mainf = M_IDLE;
				break;
					
			case M_LIGHT_FOLLOW:
			//Entered when light follow command received from PC
				mfTrackLight(&sys);
				break;
				
			case M_FORMATION:
			//placeholder
				break;
			
			case M_CHARGING:
				chargeCycleReturn = cfChargeCycleHandler(&sys);
				if(chargeCycleReturn > 0xEF)
					sys.states.mainf = M_IDLE;			//Charging fault occurred
				if(!chargeCycleReturn)
					sys.states.mainf = sys.states.mainfPrev;	//Charge finished successfully
				break;
				
			case M_TEST_ALL:
			//Something
				break;
				
			case M_OBSTACLE_AVOIDANCE:
			//OA stuff
				break;
				
			case M_IDLE:					
				mfStopRobot(&sys);
				if(!fdelay_ms(1000))					//Blink LED 3 in Idle mode
					led3Tog;	
				break;
		}
		
		xbeeGetNew();			//Checks for and interprets new communications
		nfRetrieveNavData(&sys);	//checks if there is new navigation data and updates sys->pos.
		//check to see if obstacle avoidance is enabled AND the robot is moving
		if(sys.flags.obaEnabled && sys.flags.obaMoving)
			dodgeObstacle(&sys); //avoid obstacles using proximity sensors
	}
}