/**************************************************************************/
/*!
    @file     angle_moving_avg.ino
    @author   SOSAndroid (E. Ha.)
    @license  BSD (see license.txt)

	read over I2C bus and averaging angle

    @section  HISTORY

    v1.0 - First release
*/
/**************************************************************************/


#include <ams_as5048b.h>
#include <Wire.h>

//unit consts
#define U_RAW 1
#define U_TRN 2
#define U_DEG 3
#define U_RAD 4
#define U_GRAD 5
#define U_MOA 6
#define U_SOA 7
#define U_MILNATO 8
#define U_MILSE 9
#define U_MILRU 10

AMS_AS5048B mysensor;

void setup() {

	//Start serial
	Serial.begin(9600);
	while (!Serial) ; //wait until Serial ready

	//Start Wire object. Unneeded here as this is done (optionnaly) by AMS_AS5048B object (see lib code - #define USE_WIREBEGIN_ENABLED)
	//Wire.begin();

	//init AMS_AS5048B object
	mysensor.begin();

	//set clock wise counting
	mysensor.setClockWise(true); 

	//set the 0 to the sensorr
	//mysensor.zeroRegW(0x0);

}

void loop() {

	//prints to serial the read angle in degree and its average every 2 seconds
	//prints 2 times the exact same angle - only one measurement
	mysensor.updateMovingAvgExp();


	Serial.print("Angle degree : ");
	Serial.println(mysensor.angleR(U_DEG, false), DEC);

	Serial.print("Average ");
	Serial.println(mysensor.getMovingAvgExp(U_DEG), DEC);

	delay(2000);

}
