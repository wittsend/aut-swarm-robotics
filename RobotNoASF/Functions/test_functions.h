/*
 * test_functions.h
 *
 * Created: 13/07/2017 4:55:41 PM
 *  Author: TODO:COMMECNTING AND HEADERSadams
 */ 

#ifndef TESTFUNCTIONS_H_
#define TESTFUNCTIONS_H_

//0xE0 reserved
#define TEST_COMMUNICATIONS		0xE1
//0xE2 reserved
//0xE3 reserved
#define TEST_PROXIMITY_SENSORS	0xE4
#define TEST_LIGHT_SENSORS		0xE5
#define TEST_MOTORS				0xE6
#define TEST_MOUSE_SENSOR		0xE7
#define TEST_IMU_GYRO			0xE8
#define TEST_LINE_FOLLOWERS		0xE9
//0xEA reserved
#define TEST_TWI_MULTIPLEXOR	0xEB
//0xEC reserved
#define TEST_CAMERA				0xED
//0xEE reserved
//0xEF reserved

#define DATA_RETURN				0x00
#define SINGLE_SAMPLE			0x01
#define STREAM_DATA				0x02
#define STOP_STREAMING			0xFF

#define REAR_MOTOR				0x01
#define F_RIGHT_MOTOR			0x02
#define F_LEFT_MOTOR			0x03



union float_bytes {
	float val;
	unsigned char bytes[sizeof(float)];
}floatBytesData;

struct MessageInfo;	//Defined in xbee_driver.h

/*
* Function: uint8_t testManager(struct transmitDataStructure *transmit,	RobotGlobalStructure *sys)
*
* Handles the interpretation of received test commands,
* calling the appropriate test functions / performing tests
* and returning to the PC the test return value
*
* Input is the message structure from the received data
* after the XBee framing has been stripped
*
* No Return Values
*
* Called in the main loop whenever a new command is received
* OR in the case of streaming data it will be called at every streaming interval
* ***Streaming Interval = 100ms***
*
*/
void getTestData(struct transmitDataStructure *transmit, RobotGlobalStructure *sys);

void testManager(RobotGlobalStructure *sys);



/*
* Function: void testAll(struct transmitDataStructure *transmit, RobotGlobalStructure *sys)
*
* tests all the peripherals in a set order and returns them all back to the GUI in one packet
* calling the appropriate test functions / performing tests
* and returning to the PC the test return values
*
* Input is the transmit array
*
* No Return Values
*
*/
void testAll(struct transmitDataStructure *transmit, RobotGlobalStructure *sys);








#endif /* TESTFUNCTIONS_H_ */