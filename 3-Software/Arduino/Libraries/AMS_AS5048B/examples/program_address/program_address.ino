/**************************************************************************/
/*!
    @file     program_address.ino
    @author   brentyi
    @license  BSD (see license.txt)

	Example sketch for programming the i2c slave address
	Be careful -- this can only be done once!

    @section  HISTORY

    v1.0.4 - Added address programming example
*/
/**************************************************************************/

#include <ams_as5048b.h>

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

// Construct our AS5048B object with an I2C address of 0x40
AMS_AS5048B mysensor(0x40);

void setup() {

	//start serial
	Serial.begin(9600);
	while (!Serial) ; //wait until Serial ready

	// Initialize our sensor
	mysensor.begin();

	// Set the first five MSBs, where the MSB will be internally inverted
	// If A1 & A2 are pulled low, our new address should be 0b1000100
	mysensor.addressRegW(0x01);

	// Burn our new address to the OTP register
	mysensor.doProg();
}

void loop() {

	// Print out angle readings -- this should work if programming succeeded
	Serial.print("Angle degree : ");
	Serial.println(mysensor.angleR(U_DEG, true), DEC);

	delay(200);
}
