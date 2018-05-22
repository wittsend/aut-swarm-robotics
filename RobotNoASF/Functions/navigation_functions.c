/*
* navigation_functions.h
*
* Author : Matthew Witt (pxf5695@autuni.ac.nz)
* Created: 18/08/2017 9:21:05 AM
*
* Project Repository: https://github.com/wittsend/aut-swarm-robotics
*
* Holds functions that periodically retrieve data from the navigation sensors (IMU and mouse) and
* store them in a global data structure for easy access by other parts of the program. Also
* contains functions for converting data from one form to another.
*
* More Info:
* Atmel SAM 4N Processor Datasheet:http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
* Relevant reference materials or datasheets if applicable
*
* Functions:
* uint8_t nfRetrieveNavData(RobotGlobalStructure *sys)
* void nfGetEulerAngles(RobotGlobalStructure *sys)
* float nfWrapAngle(float angleDeg)
* void nfDMPEnable(char enable RobotGlobalStructure *sys)
* void nfApplyPositionUpdateFromPC(uint8_t *rawData, RobotGlobalStructure *sys)
*
*/

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "../robot_setup.h"

#include "../IMU-DMP/inv_mpu_CUSTOM.h"

#include "../Interfaces/pio_interface.h"	//temp used by nfOpticalTesting
#include "../Interfaces/imu_interface.h"
#include "../Interfaces/external_interrupt.h"//For controlling IMU interrupt
#include "../Interfaces/opt_interface.h"
#include "../Interfaces/timer_interface.h"	//temp used by nfOpticalTesting

#include "motion_functions.h"				//Temp used by nfOpticalTesting
#include "navigation_functions.h"
#include "comm_functions.h"

#include <math.h>				//Required for round() in nfProcessOpticalData()
#include <tgmath.h>				//Required for atan2 in nfGetEulerAngles()

//////////////[Defines]/////////////////////////////////////////////////////////////////////////////

//////////////[Global variables]////////////////////////////////////////////////////////////////////
float accelxData[4096];
float accelyData[4096];
float timeData[4096];
bool motorRunning[4096];
uint16_t bufferPt = 0;


//////////////[Private Functions]///////////////////////////////////////////////////////////////////
void nfProcessAccelerometer(RobotGlobalStructure *sys);


//////////////[Functions]///////////////////////////////////////////////////////////////////////////
/*
* Function:
* uint8_t nfRetrieveNavData(RobotGlobalStructure *sys)
*
* Checks if the IMU's FIFO interrupt flag has been set, and if so, will read data from the IMU's
* FIFO buffer, convert the retrieved quaternions to Euler angles and store them and retrieve data
* from the mouse sensor and store it.
*
* Inputs:
* none
*
* Returns:
* 0 if data was retrieved (sys.flags.imuCheckFifo flag was set), otherwise returns 1.
*
* Implementation:
* imuReadFifo() reads the data from the IMU's FIFO buffer and stores the data in sys->pos..
* nfGetEulerAngles() takes the quaternion data stored in sys->pos. and converts it to useful
* Euler angles (yaw, pitch and roll) and stores it back in sys->pos..
* getMouseXY() retrieves latest data from the mouse sensor and stored it in sys->pos..
*
*/
uint8_t nfRetrieveNavData(RobotGlobalStructure *sys)
{
	if(sys->flags.imuCheckFifo)
	{
		if(sys->pos.IMU.pollEnabled)				//If polling enabled for IMU
		{
			if(!sys->pos.IMU.dmpEnabled)			//If DMP was disabled then enable it
				nfDMPEnable(1, sys);
			imuReadFifo(sys);						//Read IMU's FIFO buffer
			nfGetEulerAngles(sys);					//Convert IMU quaternions to Euler angles
			
			if(!sys->pos.Optical.pollEnabled)		//Use accelerometer to calc position if no mouse
				nfProcessAccelerometer(sys);
		} else {
			if(sys->pos.IMU.dmpEnabled)				//If polling IMU disabled, then Disable DMP
				nfDMPEnable(0, sys);
		}

		sys->flags.imuCheckFifo = 0;				//Reset interrupt flag
		
		if(sys->pos.Optical.pollEnabled)
		{
			getMouseXY(sys);						//Update mouse sensor data
			sys->pos.Optical.x += sys->pos.Optical.dx;
			sys->pos.Optical.y += sys->pos.Optical.dy;
			nfProcessOpticalData(sys);
		}
		
		
		return 0;
	} else
		return 1;
}

/*
* Function: void nfGetEulerAngles(RobotGlobalStructure *sys)
*
* Convert Quaternion numbers from the IMU to Euler rotational angles
*
* Inputs:
* RobotGlobalStructure *sys
*   Holds the address to the global sys->pos. structure that holds all positional data
*
* Returns:
* Loads Yaw, Pitch and Roll data back into sys->pos..
*
* Implementation:
* After the quaternions have been converted to Euler angles, the Yaw offset is applied which is a
* heading correction obtained from the PC. Once this has been applied, the Yaw value is checked to
* ensure it is still in range (-180<Yaw<180) and corrected if necessary.
*
*/
void nfGetEulerAngles(RobotGlobalStructure *sys)
{
	float w = sys->pos.IMU.qw;				//Pull quaternions from IMU
	float x = sys->pos.IMU.qx;
	float y = sys->pos.IMU.qy;
	float z = sys->pos.IMU.qz;
	float sqw = w*w;						//Pre-calculate squares
	float sqx = x*x;
	float sqy = y*y;
	float sqz = z*z;
	float unit = sqx + sqy + sqz + sqw;	//Should equal 1, otherwise is correction factor
	float test = x*y + z*w;
	if (test > 0.499*unit)					// singularity at north pole
	{
		sys->pos.IMU.roll = 2 * atan2(x,w);
		sys->pos.IMU.pitch = M_PI/2;
		sys->pos.IMU.yaw = 0;
		return;
	}
	if (test < -0.499*unit)					// singularity at south pole
	{
		sys->pos.IMU.roll = -2 * atan2(x,w);
		sys->pos.IMU.pitch = M_PI/2;
		sys->pos.IMU.yaw = 0;
		return;
	}
	sys->pos.IMU.roll = (atan2(2*y*w-2*x*z , sqx - sqy - sqz + sqw))*180/M_PI;
	sys->pos.IMU.pitch = (asin(2*test/unit))*180/M_PI;
	sys->pos.IMU.yaw = (atan2(2*x*w-2*y*z , -sqx + sqy - sqz + sqw))*180/M_PI;
	//Factor in the Yaw offset (Heading correction from the PC) and store in pos.facing
	sys->pos.facing = sys->pos.IMU.yaw + sys->pos.facingOffset;
	//Wrap facing so its always between -180 and 180 degrees
	sys->pos.facing = nfWrapAngle(sys->pos.facing);
}

/*
* Function:
* void nfProcessOpticalData(RobotGlobalStructure *sys)
*
* Performs processing on optical mouse data to retrieve real absolute x and y and heading 
* information
*
* Inputs:
* RobotGlobalStructure *sys
*   Pointer to the global robot data structure which is where the mouse data and calculated data is
*   is retrieved and stored
*
* Returns:
* none
*
* Implementation:
* TODO:implementation
*
* Improvements:
* [Ideas for improvements that are yet to be made](optional)
*
*/
void nfProcessOpticalData(RobotGlobalStructure *sys)
{
	//Calculate headings (deg) if there is fresh data from the optical sensor
	if(abs(sys->pos.Optical.dx) > 0 || abs(sys->pos.Optical.dy) > 0)
	{
		//Relative heading
		sys->pos.relHeading = atan2(sys->pos.Optical.dx, sys->pos.Optical.dy)*180/M_PI;
		//Absolute heading
		sys->pos.heading = sys->pos.facing + sys->pos.relHeading;
		//Wrap to within -180 to 180 deg
		sys->pos.heading = nfWrapAngle(sys->pos.heading);
	}
	
	//Calculate dx and dy in mm
	sys->pos.dx = (int32_t)((float)sys->pos.Optical.dx/sys->pos.deltaTime*1000*sys->pos.Optical.convFactor);
	sys->pos.dy = (int32_t)((float)sys->pos.Optical.dy/sys->pos.deltaTime*1000*sys->pos.Optical.convFactor);
	
	//Integrate absolute x and y in mm
	sys->pos.x += sys->pos.dx;
	sys->pos.y += sys->pos.dy;
	
	//Calculate speed from optical sensor (mm/s)
	sys->pos.speed = sqrt((sys->pos.dx*sys->pos.dx + sys->pos.dy*sys->pos.dy))
						/sys->pos.deltaTime*1000;
}

//TODO: Function comment header
void nfProcessAccelerometer(RobotGlobalStructure *sys)
{
	#define BUFF_LEN 20
	static float accelXBuffer[BUFF_LEN];
	static float accelYBuffer[BUFF_LEN];
	float accelXMean = 0;
	float accelYMean = 0;
	static uint8_t buffWPointer = 0;
	uint8_t buffRPointer;
	
	//Calculate accelerometer biases from attitude data. Removes the gravitational vector.
	sys->pos.IMU.accelXBias = sys->pos.IMU.gMag*sin(M_PI*sys->pos.IMU.roll/180.0);
	sys->pos.IMU.accelYBias = sys->pos.IMU.gMag*sin(-M_PI*sys->pos.IMU.pitch/180.0);
	
	if(bufferPt < 4096 && sys->states.mainf == M_IDLE)
	{
		
		motorRunning[bufferPt] = sys->flags.obaMoving;
		
		accelxData[bufferPt] = sys->pos.IMU.accelX;
		accelyData[bufferPt] = sys->pos.IMU.accelY;
		
		if(bufferPt == 0)
		{
			timeData[bufferPt] = 0;
		}
		else
		{
			timeData[bufferPt] = (timeData[bufferPt - 1] + sys->pos.deltaTime);
		}	
		
		bufferPt++;
	}
	
	//Calculate biased acceleration values
	//sys->pos.IMU.accelX = (sys->pos.IMU.accelX + sys->pos.IMU.accelXBias);
	//sys->pos.IMU.accelY = (sys->pos.IMU.accelY + sys->pos.IMU.accelYBias);


	
	//Add current value to back buffer and calc rolling mean.
	accelXBuffer[buffWPointer] = sys->pos.IMU.accelX;
	accelYBuffer[buffWPointer] = sys->pos.IMU.accelY;
	buffWPointer++;
	buffRPointer = buffWPointer + 10;
	if(buffWPointer > BUFF_LEN - 1) buffWPointer = 0;
	if(buffRPointer > BUFF_LEN - 1) buffRPointer -= BUFF_LEN;
	for(unsigned int i = 0; i < BUFF_LEN; i++)
	{
		accelXMean += accelXBuffer[i];
		accelYMean += accelYBuffer[i];
	}
	accelXMean /= (float)(BUFF_LEN);
	accelYMean /= (float)(BUFF_LEN);
	
	//Create accel values with removed DC
	sys->pos.IMU.accelHPFX = accelXBuffer[buffRPointer] - accelXMean;
	sys->pos.IMU.accelHPFY = accelYBuffer[buffRPointer] - accelYMean;
	
	//if(fabs(sys->pos.IMU.accelHPFX) < 0.1) sys->pos.IMU.accelHPFX = 0;
	//if(fabs(sys->pos.IMU.accelHPFY) < 0.1) sys->pos.IMU.accelHPFY = 0;
	
	sys->pos.dx += (sys->pos.IMU.accelHPFX*(sys->pos.deltaTime)/1000.0);
	sys->pos.dy += (sys->pos.IMU.accelHPFY*(sys->pos.deltaTime)/1000.0);
	
	if(fabs(sys->pos.dx) < 0.001) sys->pos.dx = 0.;
	if(fabs(sys->pos.dy) < 0.001) sys->pos.dy = 0.;
	
	sys->pos.x += (sys->pos.dx*(float)(sys->pos.deltaTime)/1000.0);
	sys->pos.y += (sys->pos.dy*(float)(sys->pos.deltaTime)/1000.0);
	
	return;
}

//TODO: Function comment header
uint16_t nfCalcAccelerometerBias(RobotGlobalStructure *sys)
{
	enum acbStates {START, CALC, FINISH};
	static FDelayInstance delay;
	static float magSum = 0;
	static uint32_t oldTimeStamp = 0;
	static uint32_t samples = 0;
	static enum acbStates state = START;	
	uint32_t delayPeriod = 13000;
	
	switch(state)
	{
		case START:
			state = CALC;
			break;
			
		case CALC:
			//First ensure than fresh data has been fetched from the IMU
			if(sys->pos.timeStamp != oldTimeStamp)
			{
				extDisableIMUInt;
				//Take sum of accelerometer magnitudes over delayPeriod(ms)
				magSum += sqrt(sys->pos.IMU.accelX*sys->pos.IMU.accelX
				+ sys->pos.IMU.accelY*sys->pos.IMU.accelY
				+ sys->pos.IMU.accelZ*sys->pos.IMU.accelZ);
				extEnableIMUInt;
				//Count the total number of samples
				samples++;
				oldTimeStamp = sys->pos.timeStamp;
			}
			//After 13secs move to next state (13s because thats how long gyro cal takes normally)
			if(sys->startupDelay > delayPeriod)
			delayPeriod = sys->startupDelay;
			if(!fdelay_ms(&delay, delayPeriod))
			state = FINISH;
			break;
			
		case FINISH:
			sys->pos.IMU.gMag = magSum/(float)samples;
			magSum = samples = 0;
			sys->pos.dx = 0;
			sys->pos.dy = 0;
			sys->pos.x = 0;
			sys->pos.y = 0;
			state = START;
			break;
	}
	
	return state;
}

/*
* Function:
* float nfWrapAngle(float angleDeg)
*
* Will take any angle in degrees and convert it to its equivalent value between -180 and 180 degrees
*
* Inputs:
* float angleDeg
*   Angle to wrap
*
* Returns:
* Wrapped equivalent of the given angle
*
* Implementation:
* TODO:Uses modulus to return the remainder of the given angle divided by 180. If the given angle 
* was less than -180 then this is the new angle. Otherwise if the original angle is greater than 180
* then the remainder has 180 subtracted from it and this becomes the new value. In any other case
* (Which is just if the input angle is less than 180 and greater than -180) just return the input
* value because it is already in range.
*
*/
float nfWrapAngle(float angleDeg)
{
	while(angleDeg > 180.0)
		angleDeg -= 360.0;
	while(angleDeg < -179.99)
		angleDeg += 360.0;
	return angleDeg;
}

/*
* Function:
* void nfDMPEnable(RobotGlobalStructure *sys)
*
* Enables the DMP on the IMU and resets the sys.pos.IMU.dmpEnabled flag. Provides a wrapper to
* enable the DMP
*
* Inputs:
* char enable:
*   1 to enable DMP and 0 to disable;
* RobotGlobalStructure *sys:
*   Pointer to the global robot data structure
*
* Returns:
* None
*
* Implementation:
* Calls the DMP enable function in the IMU driver, and sets dmpEnabled high
*
*/
void nfDMPEnable(char enable, RobotGlobalStructure *sys)
{
	if(enable)
		imuDmpStart();
	else
		imuDmpStop();
	sys->pos.IMU.dmpEnabled = enable;
}

/*
* Function:
* void nfApplyPositionUpdateFromPC(uint8_t *rawData, RobotGlobalStructure *sys)
*
* Takes the raw data buffer containing position information and updates the robots current position
*
* Inputs:
* uint8_t *rawData
*   Pointer to the data buffer array retrieved from the xbee
* RobotGlobalStructure *sys
*   Pointer to the global robot data structure
*
* Returns:
* none
*
* Implementation:
* Three 16bit unsigned integers representing the position of the robot in the arena in mm are sent
* The first is x, the second is y and the third is a facing in degrees. The coordinate system of 
* the web cam has the origin in the top left corner. On the robot however the origin would be the
* bottom left corner. To bandage this, the y value is converted to negative (and the robot thinks
* its working in the lower right quadrant)
*
* Improvements:
* Find a better way to handle coordinated between the robot and PC
*
*/
void nfApplyPositionUpdateFromPC(RobotGlobalStructure *sys)
{
	float distMouse = 0;
	float distPC = 0;
	
	int32_t deltaMouseX = 0;
	int32_t deltaMouseY = 0;
	int32_t deltaPCX = 0;
	int32_t deltaPCY = 0;

	//Update position (x and y swapped to convert coord systems. Temporary fix)
	sys->pos.y = (uint16_t)((sys->comms.xbeeMessageData[0]<<8)|sys->comms.xbeeMessageData[1]);
	sys->pos.x = (uint16_t)((sys->comms.xbeeMessageData[2]<<8)|sys->comms.xbeeMessageData[3]);
	//Update facing (only if robot isn't rotating too fast)
	if(abs(sys->pos.IMU.gyroZ) < 10)
		imuApplyYawCorrection((int16_t)((sys->comms.xbeeMessageData[4]<<8)|sys->comms.xbeeMessageData[5]), sys);
	
	if(sys->pos.oldPCX != 0 && sys->pos.oldPCY != 0)
	{
		deltaMouseX = (sys->pos.Optical.x - sys->pos.Optical.xOld);
		deltaMouseY = (sys->pos.Optical.y - sys->pos.Optical.yOld);
		deltaPCX = (sys->pos.x - sys->pos.oldPCX);
		deltaPCY = (sys->pos.y - sys->pos.oldPCY);
		
		//Distance travelled between updates in mouse counts
		distMouse = sqrt(deltaMouseX*deltaMouseX + deltaMouseY*deltaMouseY);
	
		//Distance travelled between updates in mm
		distPC = sqrt(deltaPCX*deltaPCX + deltaPCY*deltaPCY);
	} else {
		sys->pos.oldPCX = sys->pos.x;
		sys->pos.oldPCY = sys->pos.y;
	
		sys->pos.Optical.xOld = sys->pos.Optical.x;
		sys->pos.Optical.yOld = sys->pos.Optical.y;	
	}
	
	if((distPC > 80) && (distMouse))	
	{
		sys->pos.Optical.convFactor = (distPC/distMouse);
		//If this is the first time, then don't average.
		if(sys->pos.Optical.convFactor != 0)
		{
			sys->pos.Optical.convFactor /= 2.0;
		}
		sys->pos.oldPCX = sys->pos.x;
		sys->pos.oldPCY = sys->pos.y;
	
		sys->pos.Optical.xOld = sys->pos.Optical.x;
		sys->pos.Optical.yOld = sys->pos.Optical.y;
	}
	

	
	sys->flags.posPCNewData = 1;
}

uint8_t nfOpticalTesting(uint8_t speed, uint8_t distance, RobotGlobalStructure *sys)
{
	static uint8_t state = 0;
	static FDelayInstance delay;

	switch(state)
	{
		case 1:
			if(!mfMoveToHeadingByDistance(0, speed, distance, sys))
			{
				state = 2;
			}
			break;
			
		case 2:
			if(!fdelay_ms(&delay, 2000))
				state = 3;
			break;
			
		case 3:
			if(!mfMoveToHeadingByDistance(180, speed, distance, sys))
			{
				state = 4;	
			}
		
			break;
			
		case 4:
				if(!fdelay_ms(&delay, 2000))
					state = 0;
			break;
			
		case 0:
			state = 1;
			break;
	}
	return state;
}