/**************************************************************************/
/*!
    @file     wind_vane.ino
    @author   SOSAndroid (E. Ha.)
    @license  BSD (see license.txt)

	High resolution wind vane
	Measure azimuth & compass directions using an AMS AS5048B chip
	Outputs measurement to Serial every 5 seconds
	Updates measurement every seconds
	Uses Timer lib - https://github.com/JChristensen/Timer
	
    @section  HISTORY

    v1.0 - First release
    v1.0.1 - Code modification to comply with Timer.h used by Codebender.cc vs https://github.com/JChristensen/Timer
	v1.0.2 - remove duplicate code for Timer.h supported by Codebender.cc
	v1.0.3 - Azimuth calculation robustness / error handling
*/
/**************************************************************************/

#define CODEBENDER //comment if used in the Arduino IDE - deals with the Timer.h compatibility.

#include <ams_as5048b.h>
#include <Wire.h>
#include <Timer.h>
#include <math.h>

//north position of the sensor/magnet. To be adaoted according to your setting
#define NORTH 0x0

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

//magnetic rotary sensor object
AMS_AS5048B mysensor;

//Timer object
Timer mytimer;

void setup() {

	//Start serial
	Serial.begin(9600);
	while (!Serial) ; //wait until Serial ready
	
	Serial.println("Startup...");
	
	//Start Wire object. Unneeded here as this is done (optionnaly) by AMS_AS5048B object (see lib code - #define USE_WIREBEGIN_ENABLED)
	//Wire.begin();

	//init AMS_AS5048B object
	mysensor.begin();
	
    //set clock wise counting as compass truns clockwise
    mysensor.setClockWise(true); 
    
    //set sensor's North position - depending on your setting
    mysensor.zeroRegW(NORTH); //needed only once ever

#ifdef CODEBENDER
	//To comply with the Timer.h lib used by codebender.cc
	mytimer.every(1000, updateAvg_alt, (void*)0); //updates angular Exp moving average every second
	mytimer.every(5000, printWindDir_alt, (void*)0); //outputs to serial wind direction every 5 seconds
#else
	//To comply https://github.com/JChristensen/Timer
	mytimer.every(1000, updateAvg); //updates angular Exp moving average every second
	mytimer.every(5000, printWindDir); //outputs to serial wind direction every 5 seconds
#endif
    
}

void loop() {
	mytimer.update(); // updates the timer to trigger proper functions
}


char* degreeToCompass(double angle) {
	//working with angles in degree
	//returns one of the 16 compass azimuth
												//French
	if (angle < 0.0) return "ERROR";
	else if (angle < 11.25) return "N";			// N
	else if (angle < 33.75) return "NNE";		// NNE
	else if (angle < 56.25) return "NE";		// NE
	else if (angle < 78.75) return "ENE";		// ENE
	else if (angle < 101.25) return "E";		// E
	else if (angle < 123.75) return "ESE";		// ESE
	else if (angle < 146.25) return "SE";		// SE
	else if (angle < 168.75) return "SSE";		// SSE
	else if (angle < 191.25) return "S";		// S
	else if (angle < 213.75) return "SSW";		// SSO
	else if (angle < 236.25) return "SW";		// SO
	else if (angle < 258.75) return "WSW";		// OSO
	else if (angle < 281.25) return "W";		// O
	else if (angle < 303.75) return "WNW";		// ONO
	else if (angle < 326.25) return "NW";		// NO
	else if (angle < 348.75) return "NNW";		// NNO
	else if (angle <= 360.0) return "N";		// N

	return "ERROR";
}

void updateAvg (void) {
	mysensor.updateMovingAvgExp();
	return;
}

void printWindDir(void) {

	//double angle = (trunc(mysensor.getMovingAvgExp(U_DEG) * 100.0)) / 100.0 ; //rounds to 2 decimal - almost useless
	double angle = mysensor.getMovingAvgExp(U_DEG);
	
	char* compassDir[6];
	compassDir[6]  = degreeToCompass(angle);
	
	Serial.println("Wind direction");
	Serial.print("Degree: ");
	Serial.println(angle, DEC);
	Serial.print("Compass: ");
	Serial.println(compassDir[6]);
	Serial.println("----");
	
	return;
}


#ifdef CODEBENDER
//Quick fix to get the example compiling against Timer.h used by Codebender
//Dirty one, but not time to make it smarter

	void updateAvg_alt (void *context) {
		updateAvg();
		return;
	}
	
	void printWindDir_alt(void *context) {
		printWindDir();
		return;
	}

#endif