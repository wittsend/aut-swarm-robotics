/*
* robot_setup.c
*
* Author : Adam Parlane, Matthew Witt
* Created: 6/7/2017
*
* Project Repository: https://github.com/wittsend/aut-swarm-robotics
*
* Contains general/miscellaneous  functions
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

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "robot_setup.h"
#include "Interfaces/adc_interface.h"
#include "Interfaces/external_interrupt.h"
#include "Interfaces/fc_interface.h"
#include "Interfaces/imu_interface.h"
#include "Interfaces/light_sens_interface.h"
#include "Interfaces/line_sens_interface.h"
#include "Interfaces/motor_driver.h"
#include "Interfaces/opt_interface.h"
#include "Interfaces/pio_interface.h"
#include "Interfaces/prox_sens_interface.h"
#include "Interfaces/rgbled_driver.h"
#include "Interfaces/timer_interface.h"
#include "Interfaces/twimux_interface.h"
#include "Interfaces/uart_interface.h"
#include "Interfaces/xbee_driver.h"
#include "Interfaces/camera_buffer_interface.h"
#include "Interfaces/camera_interface.h"

//FOR performSystemTasks()
#include "Functions/navigation_functions.h"
#include "Functions/sensor_functions.h"
#include "Functions/power_functions.h"
#include "Functions/comm_functions.h"
#include "Functions/motion_functions.h"
//

#include <stdlib.h>				//srand()
#include <math.h>
#include <string.h>

#include <stdio.h>				//For printf

//////////////[Global variables]////////////////////////////////////////////////////////////////////
static double PRECISION = 0.00000000000001;		//Used by dtoa()

//////////////[Global variables]////////////////////////////////////////////////////////////////////
/*Global Data Structure Initialisation. These are the initial settings for the robot
Current Layout Tree:
--------------------
+sys
	+flags
		+xbeeNewData
		+imuCheckFifo
		+obaMoving
		+obaEnabled
	+states
		+mainf
		+mainfPrev
		+docking
		+chargeCycle
		+followLine
		+scanBrightest
		+moveHeadingDistance
	+pos
		+Optical
			+dx
			+dy
			+heading
			+speed
		+IMU
			+qw
			+qx
			+qy
			+qz
			+accelX
			+accelY
			+gyroX
			+gyroY
			+gyroZ
			+pitch
			+roll
			+yaw
		+x
		+y
		+heading
		+facing
		+facingOffset
		+timeStamp	
		+deltaTime
		+systemTimeStamp
		+targetHeading
		+targetSpeed
	+timeStamp
*/
RobotGlobalStructure sys =
{
	//System flags
	.flags =
	{
		.imuCheckFifo				= 0,
		.camBufferRead				= 0,
		.obaMoving					= 0,
		.obaEnabled					= 0,
		.posPCNewData				= 0
	},
	
	//System State Machine Initial States
	.states =
	{
		.mainf						= M_INITIALISATION,	//Always set to initialisation
		.mainfPrev					= M_IDLE,
		.dockingLight				= DS_START,
		.dockingCam					= DCS_START,
		.chargeCycle				= CCS_CHECK_POWER,
		.followLine					= FLS_START,
		.scanBrightest				= SBS_FUNCTION_INIT,
		.moveHeadingDistance		= MHD_START
	},
		
	//Communications
	.comms =
	{
		.twi2SlavePollEnabled		= false,	// Flag to enable or disable checking of slave requests on twi2 (from top mounted LCD)
		.twi2SlavePollInterval		= 199,		// Interval at which to poll at (ms)
		
		.pcUpdateEnable				= true,		// Flag to enable or disable PC update/status messages
		.pcUpdateInterval			= 5003,		// Interval at which the PC is updated (ms)
		.testModeStreamInterval		= 149,		// Interval between streaming test data packets back to the PC (ms)
		.xbeeNewData				= false,	// Flag to indicate new xbee data has been received
		.xbeeMissedMessages			= 0			// Number of xbee messages that have been missed due to not been check fast enough
	},
	
	//Sensor polling config
	.sensors =
	{
		.line =
		{
			.pollEnabled			= 1,	//Enable line sensor polling
			.pollInterval			= 101
		},
		
		.colour =
		{
			.pollEnabled			= 0x03,	//Bitmask to enable specific sensors. (0x03 for both)
			.pollInterval			= 41,
			.getHSV					= 1
		},
		
		.prox =
		{
			.errorCount				= 0,
			.pollEnabled			= 0x3F,		//Bitmask to enable specific sensors (0x3F for all)
			.pollInterval			= 97,
			.status					= PS_PROXIMITY,
			.setMode				= PS_PROXIMITY
		},

		.camera =
		{
			.initialised			= false
		}
	},
	
	//Robot Position data and polling config
	.pos =
	{
		.x							= 0,		//Resets robot position
		.y							= 0,		//Resets robot position
		.oldPCX						= 0,		//Used for providing correction to optical data
		.oldPCY						= 0,
		.heading					= 0.0,		//Reset heading
		.facingOffset				= 180,		//Ensures that whatever way the robot is facing when
												//powered on is 0 degrees heading.
		.targetHeading				= 0,		//Default heading is 0 degrees
		.targetSpeed				= 50,		//Default speed is 50%
		.IMU =
		{
			.pollEnabled			= 1,		//Enable IMU polling
			.pollRate				= 20,		//Sample rate from IMU. Lower this to <=10 while
												//debugging to prevent IMU overflow. Should be 200
												//for normal operation.
			.gyroCalEnabled			= 1			//Enables gyro calibration and accelerometer
												//calibration on start up so best to disable before
												//starting.
		},
		.Optical =
		{
			.pollEnabled			= 0,		//Enable Optical Polling
			.pollInterval			= 0,
			.convFactor				= 0
		}
	},
	
	.move =
	{
		.cmd						= MI_STOP,
		.heading					= 0.0,
		.speed						= 0.0,
		.dist						= 0.0,
		.facing						= 0,0,
		.maxTurnRatio				= 0,
		.x							= 0,
		.y							= 0
	},
	
	//Power/Battery/Charge
	.power =
	{
		.batteryDockingVoltage		= 3700,		//Battery voltage at which its time to find charger
		.batteryMaxVoltage			= 4050,		//Maximum battery voltage (full charge)
		.batteryMinVoltage			= 3650,		//Dead flat battery voltage
		.fcChipFaultFlag			= 0,		//Fast charge fault flag
		.pollBatteryEnabled			= 1,		//Battery polling enabled
		.pollChargingStateEnabled	= 1,		//Charge status polling enabled
		.pollChargingStateInterval	= 1009,		//Poll charging status as fast as possible
		.pollBatteryInterval		= 30011,	//Poll battery every thirty seconds
		.chargeWatchDogEnabled		= 0,		//Watchdog enabled
		.chargeWatchDogInterval		= 1013		//How often to send watchdog pulse to FC chip
	},
	
	.timeStamp						= 0,		//millisecs since power on
	.startupDelay					= 0,		//Time to wait at startup.
	.sysTaskInterval				= 5			//ms between interrupts to perform system tasks
};

//////////////[Private Functions]///////////////////////////////////////////////////////////////////
/*
* Function:
* void masterClockInit(void)
*
* Initialises the master clock to 100MHz. The master clock is the clock source that drives all the
* peripherals in the micro controller.
*
* Inputs:
* none
*
* Returns:
* none
*
*/
static void masterClockInit(void);

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
* Implementation:
* Contains functions which systematically set up each peripheral and hardware device connected to
* the robot's micro controller. Click on a function and press 'Alt + G' to open the file where it
* is kept (if using Atmel Studio)
*
* Improvements:
* Maybe
*
*/
void robotSetup(void)
{
	REG_WDT_MR = WDT_MR_WDDIS; 			//Disable system watchdog timer.
	masterClockInit();					//Initialise the master clock to 100MHz
	sysTimerInit();						//Initialise SysTick Timer
	pioInit();							//Initialise the PIO controllers
	adcSingleConvInit();				//Initialise ADC for single conversion mode
	pioLedInit();						//Initialise the LEDs on the mid board
	SPI_Init();							//Initialise SPI for talking with optical sensor
	twi0Init();							//Initialise TWI0 interface
	twi2Init();							//Initialise TWI2 interface
	lfInit();							//Initialise line follow sensors. Only on V2.
	lightSensInit(MUX_LIGHTSENS_R);		//Initialise Right Light/Colour sensor
	lightSensInit(MUX_LIGHTSENS_L);		//Initialise Left Light/Colour sensor
	proxSensInit();						//Initialise proximity sensors
	fcInit();							//Initialise the fast charge chip
	sys.sensors.camera.initialised = !(camInit()); //Initialise the camera
	imuInit();							//Initialise IMU.
	extIntInit();						//Initialise external interrupts.
	imuDmpInit(sys.pos.IMU.gyroCalEnabled, sys.pos.IMU.pollRate);//Initialise DMP system
	//mouseInit();						//Initialise mouse sensor
	xbeeInit();							//Initialise communication system
	motorInit();						//Initialise the motor driver chips
	
	//(*(__O uint32_t*)0xE000ED88U) |= (0xF<<20);
	
	sys.states.mainf = M_STARTUP_DELAY;	//DO NOT CHANGE
	
	srand(sys.timeStamp);				//Seed rand() to give unique random numbers
	
	NVIC_EnableIRQ(ID_TC2);
	return;
}

/*
* Function:
* void masterClockInit(void)
*
* Initialises the master clock to 100MHz. The master clock is the clock source that drives all the
* peripherals in the micro controller.
*
* Inputs:
* none
*
* Returns:
* none
*
* Implementation:
* - Flash Wait state must be altered to suit 100MHz master clock or else there are problems reading
*   and writing to flash.
* - Provide password to disable write protection in the PMC and clock generator registers
* - Set the start up time for the main crystal oscillator (the external 12MHz one) to 5ms. This is
*   how long the micro controller will wait before setting MOSCXTS or main crystal oscillator status
*   bit in REG_PMC_SR to 1. When it equals 1 it means the oscillator has stabilised.
* - Change the Main Clock source to the external 12MHz crystal oscillator and wait the time
*   specified earlier before continuing. The Main clock supplies a clock source to the Master clock,
*   The programmable clock controller and the phase-locked loops. The PLLs can also supply the
*   Master clock, which is what we will be doing to boost the external clock frequency from 12MHz
*   to 100MHz.
* - Disable the onboard RC oscillator which is enabled by default. No longer needed as we are now
*   running on the external crystal oscillator.
* - Set up PLLA to multiply the Main clock from 12MHz to 100MHz (12MHz/4*25 = 100MHz) and wait 63
*   slow clock cycles before continuing so that PLLA can stabilise.
* - Finally, set PLLA as the master clock source and wait for Master clock to stabilise before
*   continuing.
*
*/
void masterClockInit(void)
{
	REG_EFC_FMR
	=	EEFC_FMR_CLOE			//Opcode loop optimisation enabled
	|	EEFC_FMR_FWS(4);		//Set Flash Wait State for 100MHz (5 cycles for read write
								//operations to flash
	REG_PMC_WPMR
	=	PMC_WPMR_WPKEY(0x504D43);//Disable PMC write protect
	
	REG_CKGR_MOR
	|=	CKGR_MOR_KEY_PASSWD		//Set 5ms main xtal osc. Start up time.
	|	CKGR_MOR_MOSCXTST(255);	//Start Up Time = 8 * MOSCXTST / SLCK => MOSCXTST = 20
	
	REG_CKGR_MOR
	|=	CKGR_MOR_KEY_PASSWD		//Write enable
	|	CKGR_MOR_MOSCXTEN;		//Enable the external crystal connected to XIN and XOUT

	while(!(REG_PMC_SR & PMC_SR_MOSCXTS));//Wait for the main crystal oscillator to stabilize

	REG_CKGR_MOR
	|=	CKGR_MOR_KEY_PASSWD		//Write enable
	|	CKGR_MOR_MOSCSEL;		//MAINCK source set to external xtal

	while(!(REG_PMC_SR & PMC_SR_MOSCSELS));//Wait for the source changeover to be complete

	REG_CKGR_MOR
	=	CKGR_MOR_MOSCSEL		//Disable the RC oscillator
	|	CKGR_MOR_KEY_PASSWD
	|	CKGR_MOR_MOSCXTST(255)
	|	CKGR_MOR_MOSCXTEN;
	
	REG_CKGR_PLLAR
	|=	CKGR_PLLAR_ONE			//Must be 1 as per datasheet (pg540)
	|	CKGR_PLLAR_DIVA(3)		//Divide by 3
	|	CKGR_PLLAR_MULA(24)		//Multiply by 25 (24 + 1)
	|	CKGR_PLLAR_PLLACOUNT(63);//Wait 63 SCLK cycles before setting LOCKA bit in REG_PMC_SR	
	
	while(!(REG_PMC_SR & PMC_SR_LOCKA));//Wait for PLL LOCKA bit to be set
	
	REG_PMC_MCKR
	=	PMC_MCKR_CSS(2);		//Set PLLA_CLK as Master Clock
	
	while(!(REG_PMC_SR & PMC_SR_MCKRDY));//Wait for Master clock ready
}

void performSystemTasks(RobotGlobalStructure *sys)
{
	//There are certain states where we don't want this to run, so check we aren't in any of them
	//(For example, don't run while the hardware is being initialised)
	if(sys->states.mainf != M_INITIALISATION)
	{
		//commSendDebugFloat("LOOPED", (float) (sys->timeStamp)/1000);
		//while(sys->flags.imuCheckFifo);
		NVIC_DisableIRQ(ID_TC2);
		pfPollPower(sys);			//Poll battery and charging status
		sfPollSensors(sys);			//Poll prox, colour, line
		NVIC_EnableIRQ(ID_TC2);
		commGetNew(sys);			//Checks for and interprets new communications, but does NOT act on them.
		commPCStatusUpdate(sys);	//Updates PC with battery and state (every 5 seconds)

		//check to see if obstacle avoidance is enabled AND the robot is moving
		//if(sys.flags.obaEnabled && sys.flags.obaMoving && sys.states.mainf != M_OBSTACLE_AVOIDANCE)
		//checkForObstacles(&sys); //avoid obstacles using proximity sensors			
	}
}

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
*   The bitmask to apply to the given register.
* uint16_t timeOutMs
*   The maximum number of milliseconds to wait before exiting the function with an error.
*
* Returns:
* 0 if flag was detected or 1 if timeout was reached before flag was detected.
*
* Implementation:
* - System timestamp is loaded into a variable so we know at what time this function started.
* - A while function then waits for the flag to be set in the given register. It is also checking
*   if the time out period has been reached. The while loop exits when the flag is set or the
*   timeout period expires.
* - If the while loop exited because the flag was set, exit the function with a 0 value, otherwise
*   exit the function with a 1 (indicating an error)
*
*/
uint8_t waitForFlag(const volatile uint32_t *regAddr, uint32_t regMask, uint16_t timeOutMs)
{
	uint32_t startTime = sys.timeStamp;				//The time at which the function began
	//Wait until the desired flag is set, or until the time out period has elapsed
	while(!((*regAddr) & regMask) && (sys.timeStamp < (startTime + timeOutMs)));
	//If the flag was set (didn't time out)
	if((*regAddr) & regMask)
		return 0;
	else
		return 1;
}

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
* Implementation:
* If valueToCap is greater than maximumVal, then make it equal maximumValue, otherwise if
* valueToCap is less than minimumValue then make it equal minimum value.
*
*/
int32_t capToRangeInt(int32_t valueToCap, int32_t minimumVal, int32_t maximumVal)
{
	if(valueToCap > maximumVal)
		valueToCap = maximumVal;
	if(valueToCap < minimumVal)
		valueToCap = minimumVal;
	return valueToCap;
}

uint32_t capToRangeUint(uint32_t valueToCap, uint32_t minimumVal, uint32_t maximumVal)
{
	if(valueToCap > maximumVal)
		valueToCap = maximumVal;
	if(valueToCap < minimumVal)
		valueToCap = minimumVal;
	return valueToCap;
}

float capToRangeFlt(float valueToCap, float minimumVal, float maximumVal)
{
	if(valueToCap > maximumVal)
		valueToCap = maximumVal;
	if(valueToCap < minimumVal)
		valueToCap = minimumVal;
	return valueToCap;
}

/**
 * Double to ASCII
 */
char * dtoa(char *s, double n) {
    // handle special cases
    if (isnan(n)) {
        strcpy(s, "nan");
    } else if (isinf(n)) {
        strcpy(s, "inf");
    } else if (n == 0.0) {
        strcpy(s, "0");
    } else {
        int digit, m, m1;
        char *c = s;
        int neg = (n < 0);
        if (neg)
            n = -n;
        // calculate magnitude
        m = log10(n);
        int useExp = (m >= 14 || (neg && m >= 9) || m <= -9);
        if (neg)
            *(c++) = '-';
        // set up for scientific notation
        if (useExp) {
            if (m < 0)
               m -= 1.0;
            n = n / pow(10.0, m);
            m1 = m;
            m = 0;
        }
        if (m < 1.0) {
            m = 0;
        }
        // convert the number
        while (n > PRECISION || m >= 0) {
            double weight = pow(10.0, m);
            if (weight > 0 && !isinf(weight)) {
                digit = floor(n / weight);
                n -= (digit * weight);
                *(c++) = '0' + digit;
            }
            if (m == 0 && n > 0)
                *(c++) = '.';
            m--;
        }
        if (useExp) {
            // convert the exponent
            int i, j;
            *(c++) = 'e';
            if (m1 > 0) {
                *(c++) = '+';
            } else {
                *(c++) = '-';
                m1 = -m1;
            }
            m = 0;
            while (m1 > 0) {
                *(c++) = '0' + m1 % 10;
                m1 /= 10;
                m++;
            }
            c -= m;
            for (i = 0, j = m-1; i<j; i++, j--) {
                // swap without temporary
                c[i] ^= c[j];
                c[j] ^= c[i];
                c[i] ^= c[j];
            }
            c += m;
        }
        *(c) = '\0';
    }
    return s;
}
