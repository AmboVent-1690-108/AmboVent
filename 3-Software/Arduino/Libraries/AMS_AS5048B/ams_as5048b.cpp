/**************************************************************************/
/*!
    @file     ams_as5048b.cpp
    @author   SOSAndroid (E. Ha.)
    @license  BSD (see license.txt)

    Library to interface the AS5048B magnetic rotary encoder from AMS over the I2C bus

    @section  HISTORY

    v1.0.0 - First release
    v1.0.1 - Typo to allow compiling on Codebender.cc (Math.h vs math.h)
    v1.0.2 - setZeroReg() issue raised by @MechatronicsWorkman
	v1.0.3 - Small bug fix and improvement by @DavidHowlett
*/
/**************************************************************************/

#include <stdlib.h>
#include <math.h>
#include <Wire.h>
#include "ams_as5048b.h"

/*========================================================================*/
/*                            CONSTRUCTORS                                */
/*========================================================================*/

/**************************************************************************/
/*!
    Constructor
*/
/**************************************************************************/
AMS_AS5048B::AMS_AS5048B(void) {
	_chipAddress = AS5048_ADDRESS;
	_debugFlag = false;
}

AMS_AS5048B::AMS_AS5048B(uint8_t chipAddress) {
	_chipAddress = chipAddress;
	_debugFlag = false;
}

/*========================================================================*/
/*                           PUBLIC FUNCTIONS                             */
/*========================================================================*/

/**************************************************************************/
/*!
    @brief  init values and overall behaviors for AS5948B use

    @params
				none
    @returns
				none
*/
/**************************************************************************/

void AMS_AS5048B::begin(void) {

	#ifdef USE_WIREBEGIN_ENABLED
		Wire.begin();
	#endif
	#ifdef SERIAL_DEBUG_ENABLED
		_debugFlag = true;
		if (!Serial) {
			Serial.begin(9600);
		}
	#endif

	_clockWise = false;
	_lastAngleRaw = 0.0;
	_zeroRegVal = AMS_AS5048B::zeroRegR();
	_addressRegVal = AMS_AS5048B::addressRegR();

	AMS_AS5048B::resetMovingAvgExp();

	return;
}

/**************************************************************************/
/*!
    @brief  Toggle debug output to serial

    @params
				none
    @returns
				none
*/
/**************************************************************************/
void AMS_AS5048B::toggleDebug(void) {

	_debugFlag = !_debugFlag;
	return;
}

/**************************************************************************/
/*!
    @brief  Set / unset clock wise counting - sensor counts CCW natively

    @params[in]
				boolean cw - true: CW, false: CCW
    @returns
				none
*/
/**************************************************************************/
void AMS_AS5048B::setClockWise(boolean cw) {

	_clockWise = cw;
	_lastAngleRaw = 0.0;
	AMS_AS5048B::resetMovingAvgExp();
	return;
}

/**************************************************************************/
/*!
    @brief  writes OTP control register

    @params[in]
				unit8_t register value
    @returns
				none
*/
/**************************************************************************/
void AMS_AS5048B::progRegister(uint8_t regVal) {

	AMS_AS5048B::writeReg(AS5048B_PROG_REG, regVal);
	return;
}

/**************************************************************************/
/*!
    @brief  Burn values to the slave address OTP register

    @params[in]
				none
    @returns
				none
*/
/**************************************************************************/
void AMS_AS5048B::doProg(void) {

	//enable special programming mode
	AMS_AS5048B::progRegister(0xFD);
	delay(10);

	//set the burn bit: enables automatic programming procedure
	AMS_AS5048B::progRegister(0x08);
	delay(10);

	//disable special programming mode
	AMS_AS5048B::progRegister(0x00);
	delay(10);

	return;
}

/**************************************************************************/
/*!
    @brief  Burn values to the zero position OTP register

    @params[in]
				none
    @returns
				none
*/
/**************************************************************************/
void AMS_AS5048B::doProgZero(void) {
	//this will burn the zero position OTP register like described in the datasheet
	//enable programming mode
	AMS_AS5048B::progRegister(0x01);
	delay(10);

	//set the burn bit: enables automatic programming procedure
	AMS_AS5048B::progRegister(0x08);
	delay(10);

	//read angle information (equals to 0)
	AMS_AS5048B::readReg16(AS5048B_ANGLMSB_REG);
	delay(10);

	//enable verification
	AMS_AS5048B::progRegister(0x40);
	delay(10);

	//read angle information (equals to 0)
	AMS_AS5048B::readReg16(AS5048B_ANGLMSB_REG);
	delay(10);

	return;
}

/**************************************************************************/
/*!
    @brief  write I2C address value (5 bits) into the address register

    @params[in]
				unit8_t register value
    @returns
				none
*/
/**************************************************************************/
void AMS_AS5048B::addressRegW(uint8_t regVal) {

	// write the new chip address to the register
	AMS_AS5048B::writeReg(AS5048B_ADDR_REG, regVal);

	// update our chip address with our 5 programmable bits
	// the MSB is internally inverted, so we flip the leftmost bit
	_chipAddress = ((regVal << 2) | (_chipAddress & 0b11)) ^ (1 << 6);
	return;
}

/**************************************************************************/
/*!
    @brief  reads I2C address register value

    @params[in]
				none
    @returns
				uint8_t register value
*/
/**************************************************************************/
uint8_t AMS_AS5048B::addressRegR(void) {

	return AMS_AS5048B::readReg8(AS5048B_ADDR_REG);
}

/**************************************************************************/
/*!
    @brief  sets current angle as the zero position

    @params[in]
				none
    @returns
				none
*/
/**************************************************************************/
void AMS_AS5048B::setZeroReg(void) {

        AMS_AS5048B::zeroRegW((uint16_t) 0x00); //Issue closed by @MechatronicsWorkman and @oilXander. The last sequence avoids any offset for the new Zero position
	uint16_t newZero = AMS_AS5048B::readReg16(AS5048B_ANGLMSB_REG);
        AMS_AS5048B::zeroRegW(newZero);
	return;
}

/**************************************************************************/
/*!
    @brief  writes the 2 bytes Zero position register value

    @params[in]
				unit16_t register value
    @returns
				none
*/
/**************************************************************************/
void AMS_AS5048B::zeroRegW(uint16_t regVal) {

	AMS_AS5048B::writeReg(AS5048B_ZEROMSB_REG, (uint8_t) (regVal >> 6));
	AMS_AS5048B::writeReg(AS5048B_ZEROLSB_REG, (uint8_t) (regVal & 0x3F));
	return;
}

/**************************************************************************/
/*!
    @brief  reads the 2 bytes Zero position register value

    @params[in]
				none
    @returns
				uint16_t register value trimmed on 14 bits
*/
/**************************************************************************/
uint16_t AMS_AS5048B::zeroRegR(void) {

	return AMS_AS5048B::readReg16(AS5048B_ZEROMSB_REG);
}

/**************************************************************************/
/*!
    @brief  reads the 2 bytes magnitude register value

    @params[in]
				none
    @returns
				uint16_t register value trimmed on 14 bits
*/
/**************************************************************************/
uint16_t AMS_AS5048B::magnitudeR(void) {

	return AMS_AS5048B::readReg16(AS5048B_MAGNMSB_REG);
}

uint16_t AMS_AS5048B::angleRegR(void) {

	return AMS_AS5048B::readReg16(AS5048B_ANGLMSB_REG);
}

/**************************************************************************/
/*!
    @brief  reads the 1 bytes auto gain register value

    @params[in]
				none
    @returns
				uint8_t register value
*/
/**************************************************************************/
uint8_t AMS_AS5048B::getAutoGain(void) {

	return AMS_AS5048B::readReg8(AS5048B_GAIN_REG);
}

/**************************************************************************/
/*!
    @brief  reads the 1 bytes diagnostic register value

    @params[in]
				none
    @returns
				uint8_t register value
*/
/**************************************************************************/
uint8_t AMS_AS5048B::getDiagReg(void) {

	return AMS_AS5048B::readReg8(AS5048B_DIAG_REG);
}

/**************************************************************************/
/*!
    @brief  reads current angle value and converts it into the desired unit

    @params[in]
				String unit : string expressing the unit of the angle. Sensor raw value as default
    @params[in]
				Boolean newVal : have a new measurement or use the last read one. True as default
    @returns
				Double angle value converted into the desired unit
*/
/**************************************************************************/
double AMS_AS5048B::angleR(int unit, boolean newVal) {

	double angleRaw;

	if (newVal) {
		if(_clockWise) {
			angleRaw = (double) (0b11111111111111 - AMS_AS5048B::readReg16(AS5048B_ANGLMSB_REG));
		}
		else {
			angleRaw = (double) AMS_AS5048B::readReg16(AS5048B_ANGLMSB_REG);
		}
		_lastAngleRaw = angleRaw;
	}
	else {
		angleRaw = _lastAngleRaw;
	}

	return AMS_AS5048B::convertAngle(unit, angleRaw);
}

/**************************************************************************/
/*!
    @brief  Performs an exponential moving average on the angle.
			Works on Sine and Cosine of the angle to avoid issues 0°/360° discontinuity

    @params[in]
				none
    @returns
				none
*/
/**************************************************************************/
void AMS_AS5048B::updateMovingAvgExp(void) {

	//sine and cosine calculation on angles in radian

	double angle = AMS_AS5048B::angleR(U_RAD, true);

	if (_movingAvgCountLoop < EXP_MOVAVG_LOOP) {
		_movingAvgExpSin += sin(angle);
		_movingAvgExpCos += cos(angle);
		if (_movingAvgCountLoop == (EXP_MOVAVG_LOOP - 1)) {
			_movingAvgExpSin = _movingAvgExpSin / EXP_MOVAVG_LOOP;
			_movingAvgExpCos = _movingAvgExpCos / EXP_MOVAVG_LOOP;
		}
		_movingAvgCountLoop ++;
	}
	else {
		double movavgexpsin = _movingAvgExpSin + _movingAvgExpAlpha * (sin(angle) - _movingAvgExpSin);
		double movavgexpcos = _movingAvgExpCos + _movingAvgExpAlpha * (cos(angle) - _movingAvgExpCos);
		_movingAvgExpSin = movavgexpsin;
		_movingAvgExpCos = movavgexpcos;
		_movingAvgExpAngle = getExpAvgRawAngle();
	}

	return;
}

/**************************************************************************/
/*!
    @brief  sent back the exponential moving averaged angle in the desired unit

    @params[in]
				String unit : string expressing the unit of the angle. Sensor raw value as default
    @returns
				Double exponential moving averaged angle value
*/
/**************************************************************************/
double AMS_AS5048B::getMovingAvgExp(int unit) {

	return AMS_AS5048B::convertAngle(unit, _movingAvgExpAngle);
}

void AMS_AS5048B::resetMovingAvgExp(void) {

	_movingAvgExpAngle = 0.0;
	_movingAvgCountLoop = 0;
	_movingAvgExpAlpha = 2.0 / (EXP_MOVAVG_N + 1.0);
	return;
}


/*========================================================================*/
/*                           PRIVATE FUNCTIONS                            */
/*========================================================================*/

uint8_t AMS_AS5048B::readReg8(uint8_t address) {

	uint8_t readValue;
	byte requestResult;
	uint8_t nbByte2Read = 1;

	Wire.beginTransmission(_chipAddress);
	Wire.write(address);
	requestResult = Wire.endTransmission(false);
	if (requestResult){
		Serial.print("I2C error: ");
		Serial.println(requestResult);
	}

	Wire.requestFrom(_chipAddress, nbByte2Read);
	readValue = (uint8_t) Wire.read();

	return readValue;
}

uint16_t AMS_AS5048B::readReg16(uint8_t address) {
	//16 bit value got from 2 8bits registers (7..0 MSB + 5..0 LSB) => 14 bits value

	uint8_t nbByte2Read = 2;
	byte requestResult;
	byte readArray[2];
	uint16_t readValue = 0;

	Wire.beginTransmission(_chipAddress);
	Wire.write(address);
	requestResult = Wire.endTransmission(false);
	if (requestResult){
		Serial.print("I2C error: ");
		Serial.println(requestResult);
	}


	Wire.requestFrom(_chipAddress, nbByte2Read);
	for (byte i=0; i < nbByte2Read; i++) {
		readArray[i] = Wire.read();
	}

	readValue = (((uint16_t) readArray[0]) << 6);
	readValue += (readArray[1] & 0x3F);
	/*
	Serial.println(readArray[0], BIN);
	Serial.println(readArray[1], BIN);
	Serial.println(readValue, BIN);
	*/
	return readValue;
}

void AMS_AS5048B::writeReg(uint8_t address, uint8_t value) {

	Wire.beginTransmission(_chipAddress);
	Wire.write(address);
	Wire.write(value);
	Wire.endTransmission();

	return;
}

double AMS_AS5048B::convertAngle(int unit, double angle) {

	// convert raw sensor reading into angle unit

	double angleConv;

	switch (unit) {
		case U_RAW:
			//Sensor raw measurement
			angleConv = angle;
			break;
		case U_TRN:
			//full turn ratio
			angleConv = (angle / AS5048B_RESOLUTION);
			break;
		case U_DEG:
			//degree
			angleConv = (angle / AS5048B_RESOLUTION) * 360.0;
			break;
		case U_RAD:
			//Radian
			angleConv = (angle / AS5048B_RESOLUTION) * 2 * M_PI;
			break;
		case U_MOA:
			//minute of arc
			angleConv = (angle / AS5048B_RESOLUTION) * 60.0 * 360.0;
			break;
		case U_SOA:
			//second of arc
			angleConv = (angle / AS5048B_RESOLUTION) * 60.0 * 60.0 * 360.0;
			break;
		case U_GRAD:
			//grade
			angleConv = (angle / AS5048B_RESOLUTION) * 400.0;
			break;
		case U_MILNATO:
			//NATO MIL
			angleConv = (angle / AS5048B_RESOLUTION) * 6400.0;
			break;
		case U_MILSE:
			//Swedish MIL
			angleConv = (angle / AS5048B_RESOLUTION) * 6300.0;
			break;
		case U_MILRU:
			//Russian MIL
			angleConv = (angle / AS5048B_RESOLUTION) * 6000.0;
			break;
		default:
			//no conversion => raw angle
			angleConv = angle;
			break;
	}
	return angleConv;
}

double AMS_AS5048B::getExpAvgRawAngle(void) {

	double angle;
	double twopi = 2 * M_PI;

	if (_movingAvgExpSin < 0.0) {
		angle = twopi - acos(_movingAvgExpCos);
	}
	else {
		angle = acos(_movingAvgExpCos);
	}

	angle = (angle / twopi) * AS5048B_RESOLUTION;

	return angle;
}

void AMS_AS5048B::printDebug(void) {

	return;
}
