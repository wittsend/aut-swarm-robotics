/*
* timer_interface.h
*
* Author : Adam Parlane & Matthew Witt
* Created: 6/08/2017 1:23:27 PM
*
* Project Repository:https://github.com/wittsend/aut-swarm-robotics
*
* Sets up the timer, for the camera and 1ms interrupts
* Has delay and get ms functions and the timer handler interrupt
*
* More Info:
* Atmel SAM 4N Processor Datasheet:http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
*
* Functions:
* void sysTimerInit(void)
* int get_ms(uint32_t *timestamp)
* int delay_ms(uint32_t period_ms)
* void SysTick_Handler()
*
*/

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "../robot_setup.h"		//For performSystemTasks()
#include "timer_interface.h"
#include "pio_interface.h"
#include "../Functions/navigation_functions.h"
#include "../Functions/motion_functions.h"

//////////////[Private Defines]/////////////////////////////////////////////////////////////////////
#define SYS_CLOCK_SPD	100000000	//Clock speed in Hz

//////////////[Global Variables]////////////////////////////////////////////////////////////////////
extern RobotGlobalStructure sys;	//gives access TC interrupt handler and delay_ms(), get_ms(),
									//and fdelay_ms() access to sys.timeStamp

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
/*
* Function:
* void sysTimerInit(void)
*
* Initializes SysTick timer
* Used to time events with a 1ms interrupt
*
* Inputs:
* none
*
* Returns:
* none
*
* Implementation:
* Gives Clock access to timer counter 0 and 1
* Note	REG_TC0_CMR0 is TC0 channel 0
*		Reg_TC0_CMR1 is TC0 channel 1
*		For some reason this is a bit strange as the interrupt will only work as TC1
*		This seems to act as the interrupt for channel 1 regardless of if it is timer0 or timer1
*
* Channel 1 is for generating ms timing triggers and is set to 3.125MHz using divisor clock3
* An interrupt is setup  (named TC1_Handler) to trigger on RC compare match
* RC = 3125 therefore match every 3.125MHz / 3125 = 1kHz -> 1ms
* This channel is then enabled and the clock started
*
*/
void sysTimerInit(void)
{
	//Enable the System Tick Timer to generate an exception every millisecond.
	SysTick_Config(SYS_CLOCK_SPD/1000);
	
	//Timer Counter 0 Channel 2 Config (Used for system task triggering)
	//Enable the peripheral clock for TC2
	
	//Enable interrupts
	//NVIC_EnableIRQ(ID_TC2);				//Enable interrupts on Timer Counter 0, Ch2
	
	//REG_PMC_PCER0
	//|=	(1<<ID_TC2);
	//REG_TC0_WPMR
	//=	(0x54494D << 8);				//Disable Write Protection
	//REG_TC0_CMR2						//TC Channel Mode Register (Pg877)
	//=	TC_CMR_TCCLKS_TIMER_CLOCK3		//Prescaler MCK/32 (100MHz/32 = 3.125MHz)
	//|	TC_CMR_WAVSEL_UP_RC				//Up mode, trig on RC compare
	//|	TC_CMR_CPCSTOP					//Stop counter on RC compare
	//|	TC_CMR_WAVE;					//Waveform mode
	//REG_TC0_RC2							//RC set to 3125*5 counts
	//|=	(TC_RC_RC(3125*5));
	//REG_TC0_IER2
	//|=	TC_IER_CPCS;					//Enable the RC compare interrupt
	//REG_TC0_CCR2						//Clock control register
	//|=	TC_CCR_CLKEN					//Enable the timer clk.
	//|	TC_CCR_SWTRG;					//Start timer register counter
}

/*
* Function: int get_ms(uint32_t *timestamp)
*
* Required by the IMU drivers (hence naming convention). Outputs the system uptime generated from
* Timer0.
*
* Inputs:
* address of an integer where the timestamp will be stored
*
* Returns:
* function will return 1 if invalid pointer is passed, otherwise a 0 on success
*
* Implementation:
* Retrieves the value stored in sys.timeStamp (stores the number of millisecs that have elapsed
* since power on) and drops it at the address given by *timestamp. if *timestamp is an invalid
* address then returns a 1.
*
*/
int get_ms(uint32_t *timestamp)
{
	if(!timestamp)
		return 1;
	*timestamp = sys.timeStamp;
	return 0;
}

/*
* Function: int delay_ms(uint32_t period_ms)
*
* Halts execution for desired number of milliseconds. 
* Required by the IMU drivers (hence naming convention).
*
* Inputs:
* period_ms is the number of milliseconds to wait
*
* Returns:
* Always returns 0
*
* Implementation:
* Uses the SysTick to monitor the change in time (timeDiff). The timeDiff is summed in timeEla to
* count the amount of clock cycles that have elapsed. When the appropriate number of clock cycles
* has elapsed to constitute 1ms, period_ms is decremented until it reaches 0, at which point the 
* function exits.
*
*/
int delay_ms(uint32_t period_ms)
{
	int32_t timeOld = SysTick->VAL;
	int32_t timeCur;
	int32_t timeDiff;
	int32_t timeEla = 0;
	int32_t ticksPerMillisec = SysTick->LOAD;
	
	while(period_ms > 0)
	{
		timeCur = SysTick->VAL;
		timeDiff = (timeOld - timeCur);
		if(timeDiff < 0) timeDiff += SysTick->LOAD;
		timeEla += timeDiff;
		if(timeEla >= ticksPerMillisec)
		{
			period_ms--;
			timeEla -= ticksPerMillisec;
		}
		timeOld = timeCur;
	}
	return 0;
}

/*
* Function: int delay_us(uint32_t period_us)
*
* Halts execution for desired number of microseconds.
*
* Inputs:
* period_us is the number of milliseconds to wait
*
* Returns:
* Always returns 0
*
* Implementation:
* see delay_ms() description
*
*/
int delay_us(float period_us)
{
	int32_t timeOld = SysTick->VAL;
	int32_t timeCur;
	int32_t timeDiff;
	
	while(period_us > 0)
	{
		timeCur = SysTick->VAL;
		timeDiff = (timeOld - timeCur);
		if(timeDiff < 0) timeDiff += SysTick->LOAD;
		period_us -= timeDiff*0.01;
		timeOld = timeCur;
	}
	return 0;
}

/*
* Function:
* uint8_t fdelay_ms(uint32_t period_ms)
*
* Multi-task friendly delay
*
* Inputs:
* FDelayInstance thisDelay:
*	The instance of the delay to work with. Because this delay function is multitask friendly, it
*	means that it is possible to call this function multiple times from separate functions. If
*	There were no delay instances, then all the delays would clash with each other if they were
*	being run at the same time. The FDelayInstance should be created as a static var in the calling
*	function so that it retains its value between calls.
* uint32_t period_ms
*   Delay in ms
*
* Returns:
* 0 when time is up, otherwise 1
*
* Implementation:
* A state machine governs this timer. The start state will read off the time that the delay started
* from the system time stamp. From here it moves to the wait state. If the system time stamp isn't
* greater than the start time plus the delay period, then time isn;t up, so return 1. If the desired
* amount of time has elapsed, then move to the stop state, and reset the function. Return 0 when
* done.
*
*/
uint8_t fdelay_ms(FDelayInstance *thisDelay, uint32_t period_ms)
{
	uint32_t timeStamp;
	get_ms(&timeStamp);

	if(!period_ms)
		thisDelay->state = FD_STOP;

	switch(thisDelay->state)
	{
		case FD_START:			
			thisDelay->startTime = timeStamp;
			thisDelay->state = FD_WAIT;
		break;
		
		case FD_WAIT:
			if(timeStamp > thisDelay->startTime + period_ms)
				thisDelay->state = FD_STOP;
		break;
		
		case FD_STOP:
			thisDelay->state = FD_START;
			return 0;
		break;
	}
	return 1;
}

/*
* Function: void SysTick_Handler()
*
* System tick exception handler for get_ms, and the systemTimeStamp.
* Triggered every 1ms
* 
* Inputs:
* none
*
* Returns:
* None
*
* Implementation:
* [This description is rather old now]
* Interrupt handler for Timer Counter 1, Channel 0. As there are 6 Timer Counter Channels spread
* across two Timer Counter Modules, there are also 6 Timer Counter Interrupt handlers. Unintuitively
* these have been labeled TC0_Handler to TC6_Handler. TC3_Handler is the interrupt handler for TC1,
* Channel 0, which is the third TC channel on the device.
* Triggered every 1ms using register C compare match
* Used to help implement get_ms() and delay_ms() functions required by the IMU driver. 
* Every ms it increments the sys.timeStamp, streamDelayCounter and delaymsCounter
* streamDelayCounter is used to send test data to the PC every 100ms so this counts to 100ms
* and then sets a flag letting main() know its time to send the next set of data
* Also includes some IMU manual stuff for V1
*
*/
void SysTick_Handler()
{
	sys.timeStamp++;//used for get ms
}
