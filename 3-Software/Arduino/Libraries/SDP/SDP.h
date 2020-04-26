#include "stdint.h"
#include <cstdint>
#include <Arduino.h>
//#include "debug_utils.h"

//#ifndef DEBUG_UTILS_H
//#define DEBUG_UTILS_H
//#endif

#ifdef DEBUG
 #define DEBUG_PRINT(x)     		Serial.print (x)
 #define DEBUG_PRINTDEC(x)      Serial.print (x, DEC)
 #define DEBUG_PRINTLN(x)  			Serial.println (x)
#else
 #define DEBUG_PRINT(x)
 #define DEBUG_PRINTDEC(x)
 #define DEBUG_PRINTLN(x)
#endif

#ifdef VERBOSE
 #define VERBOSE_PRINT(x)     	Serial.print (x)
 #define VERBOSE_PRINTHEX(x)  	Serial.print(x, HEX)
 #define VERBOSE_PRINTDEC(x)  	Serial.print (x, DEC)
 #define VERBOSE_PRINTLN(x)   	Serial.println (x)
 #define VERBOSE_DELAY(x) 			delay(x)
#else
 #define VERBOSE_PRINT(x)
 #define VERBOSE_PRINTDEC(x)
 #define VERBOSE_PRINTHEX(x)
 #define VERBOSE_PRINTLN(x)
 #define VERBOSE_DELAY
#endif

#ifndef SDP_H
	#define SDP_H

	// convert two 8 bit values to one word
	#define BIU16(data, start) (((int16_t)(data)[start]) << 8 | ((data)[start + 1]))
  // access each uint8_t of a uint16_t
  #define B16TO8_1(data) (uint8_t)((data & 0xFF00) >> 8)
  #define B16TO8_2(data) (uint8_t)(data & 0x00FF)

#ifndef DEFAULT_SDP_ADDRESS
	#define DEFAULT_SDP_ADDRESS 0x25
#endif

// @brief -> Enumeration to configure the temperature compensation for
//           measurement.
typedef enum {
  SDP_TEMPCOMP_MASS_FLOW,
  SDP_TEMPCOMP_DIFFERENTIAL_PRESSURE
} SdpTempComp;

// @brief -> Enumeration to configure the averaging for measurement.
typedef enum {
  SDP_AVERAGING_NONE,
  SDP_AVERAGING_TILL_READ
} SdpAveraging;

// Enumeration to configure the clock stretching
typedef enum{
  SDP_CLKST_NONE,
  SDP_CLKST_ACTIVE
}SdpClockStretching;

typedef enum{
	IDLE = 0x00,
	CONTINUOUS_MEASUREMENT = 0x10,
	MF_NONE = 0x11,
	MF_AVG = 0x12,
	DP_NONE = 0x13,
	DP_CLKST = 0x14,
	SLEEP = 0x20
}SdpStatus;

// Enumeration of product id of each model for identification and scale factor
typedef enum{
	SDP800_500Pa = 0x03020186,
	SDP810_500Pa = 0x03020A86,
	SDP800_125Pa = 0x03020286,
	SDP810_125Pa = 0x03020B86,
	SDP31 = 0x03010188,
	SDP32 = 0x03010283
}SdpProductNumber;

typedef enum{
	ERROR_NONE = 0, // success
 	ERROR_DATA = 1, // data too long
	ERROR_NACK_ADDRESS = 2, //received NACK on transmit of address
	ERROR_NACK_DATA = 3, //received NACK on transmit of data
	ERROR_OTHER = 4, // other error
	ERROR_RECEIVE = 5, // no bytes received
}Error;

// Enumeration of I2C commands
typedef enum {
  // Undefined dummy command.
  COMMAND_UNDEFINED                       = 0x0000,
  // Start continous measurement
  // Temperature compensation: Mass flow
  // Averaging: Average till read
  COMMAND_START_MEASURMENT_MF_AVERAGE     = 0x3603,
  // Start continous measurement
  // Temperature compensation: Mass flow
  // Averaging: None - Update rate 1ms
  COMMAND_START_MEASURMENT_MF_NONE        = 0x3608,
  // Start continous measurement
  // Temperature compensation: Differential pressure
  // Averaging: Average till read
  COMMAND_START_MEASURMENT_DP_AVERAGE     = 0x3615,
  // Start continous measurement
  // Temperature compensation: Differential pressure
  // Averaging: None - Update rate 1ms
  COMMAND_START_MEASURMENT_DP_NONE        = 0x361E,
  // Stop continuous measurement.
  COMMAND_STOP_CONTINOUS_MEASUREMENT      = 0x3FF9,
	// Triggers measurement
	// Temperature compensation: mass flow
	// Clock stretching: active
  COMMAND_TRIGGER_MEASUREMENT_MF_CLK_STRETCHING = 0x3726,
	// Triggers measurement
	// Temperature compensation: mass flow
	// Clock stretching: none
  COMMAND_TRIGGER_MEASUREMENT_MF_NONE= 0x3724,
	// Triggers measurement
	// Temperature compensation: differential pressure
	// Clock stretching: active
  COMMAND_TRIGGER_MEASUREMENT_DP_CLK_STRETCHING = 0x372D,
	// Triggers measurement
	// Temperature compensation: differential pressure
	// Clock stretching: none
  COMMAND_TRIGGER_MEASUREMENT_DP_NONE = 0x362F,
	// Command for soft reset
  COMMAND_SOFT_RESET = 0x0006,
	// Pair of commands to trigger reading of product Id
  COMMAND_READ_PRODUCT_ID_1 = 0x367C,
  COMMAND_READ_PRODUCT_ID_2 = 0xE102,
	// Enter sleep mode
  COMMAND_ENTER_SLEEP_MODE = 0x3677
} Command;

class SDP_Controller{
public:
  SDP_Controller();
	void getAddress(void);
	void getProductId(void);
	void getSerialNumber(void);
	void begin(void);
	void startContinuousMeasurement(SdpTempComp tempComp, SdpAveraging averaging);
	void stopContinuousMeasurement(void);
	float getTemperatureTrigger(SdpTempComp tempComp, SdpClockStretching clkSt);
	float getDiffPressureTrigger(SdpTempComp tempComp, SdpClockStretching clkSt);
	float getTemperature(void);
	float getOnlyTemperature(void);
  float getDiffPressure(float atmPressure);
	float getDiffPressure(double atmPressure);
	float getDiffPressure(void);
	void softReset(void);
	void enterSleep(void);
	void exitSleep(void);

private:
	// Comms functions
  Error sendCommand(Command cmd);
	Error sendCommandId(Command cmd1, Command cmd2);
  Error readSequence(void);
  Error readIdSequence(void);

	// CRC functions
	bool CheckCrc(uint8_t data[], uint8_t posInit, uint8_t checksum);
	bool checkCrcRoutine(uint8_t idBuffer[]);
	
	float getPresScaleFactor(uint8_t productId[]);
	Error setError(uint8_t errorByte);

	// Data fields for each instance of the class
	float temperature;
	float diffPressure;
  uint8_t dataBuffer[9];
	uint8_t idBuffer[18];
	float scaleFactorDiffPressure;
	float scaleFactorTemperature = 200; //same value for all sensors

	uint8_t productId[4];
	uint8_t serialNumber [8];
};

#endif
