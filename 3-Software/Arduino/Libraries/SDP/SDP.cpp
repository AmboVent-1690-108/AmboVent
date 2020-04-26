#include "stdint.h"
//#include <cstdint>
#include "SDP.h"
#include "Arduino.h"
#include "Wire.h"
//#include "i2c_t3.h" //This library can be download 
#define DEBUG
//#define VERBOSE //Verbose
//#include "debug_utils.h" //Debug
SDP_Controller::SDP_Controller(){}
/* PUBLIC FUNCTIONS */
/* 
brief > search the i2c address of the DSP sensor or other sensor which is connected
param > none
return > print to serial monitor
*/
void SDP_Controller::getAddress(void){
  byte error, address; //Start Scanning i2C Address
  int nDevices;
  delay(2000);
  Serial.println();
  Serial.println("14CORE | i2C Scanner")
  delay(2000);
  Serial.println("Scanning...");
  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    Wire.beginTransmission(address); //Scanner return value of write / endTransmission to the ACK Address
    error = Wire.endTransmission();
    if (error == 0)
    {
      Serial.println("Default SDP address = 0x25");
      Serial.print("I2C device found at address 0x");
      if (address < 16)
      Serial.print("0");
      Serial.println(address,HEX);
      getProductId();
      switch(productId[2]){
        case 0x01:
          if(productId[3] == 0x86){
            Serial.print("Sensor found: ");
            Serial.println("SDP800 - 500Pa");
            this->scaleFactorDiffPressure = 60; // as default
          }
          else{
            Serial.print("Sensor found: ");
            Serial.println("SDP31 (500Pa)");
            this->scaleFactorDiffPressure = 60;
          }
        break;
        case 0x0A:
          Serial.print("Sensor found: ");
          Serial.println("SDP810 - 500Pa");
          this->scaleFactorDiffPressure = 60;
        break;
        case 0x02:
          if(productId[3] == 0x86){
            Serial.print("Sensor found: ");
            Serial.println("SDP800 - 125Pa");
            this->scaleFactorDiffPressure = 240;
          }
          else{
            Serial.print("Sensor found: ");
            Serial.println("SDP32 (125Pa)");
            this->scaleFactorDiffPressure = 240;
          }
        break;
        case 0x0B:
          Serial.print("Sensor found: ");
          Serial.println("SDP810 - 125Pa");
          this->scaleFactorDiffPressure = 240;
        break;
        default:
          Serial.println("Product ID does not match with the known variants of the sensor");
          this->scaleFactorDiffPressure = 60; 
        break;
      }
      nDevices++;
    }
    else if (error==4)
    {
      Serial.println("Default SDP address = 0x25"); // See the product manual 
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }
  }
  if (nDevices == 0){
    Serial.println("Error No I2C devices found\n");
    delay(5000);
  }
  else{
    Serial.println("Done\n");
    delay(5000);
  }
}
void SDP_Controller::getProductId(void){
  delay(100); // just in case it is called right after a softReset
  //Send and read i2c
  sendCommand(COMMAND_READ_PRODUCT_ID_1);
  sendCommand(COMMAND_READ_PRODUCT_ID_2);
  if(readIdSequence() == ERROR_NONE){
  // Copy values from buffer
  productId[0] = idBuffer[0];
  productId[1] = idBuffer[1];
  productId[2] = idBuffer[3];
  productId[3] = idBuffer[4];
  // Copy values from buffer
  serialNumber[7] = idBuffer[16];
  serialNumber[6] = idBuffer[15];
  serialNumber[5] = idBuffer[13];
  serialNumber[4] = idBuffer[12];
  serialNumber[3] = idBuffer[10];
  serialNumber[2] = idBuffer[9];
  serialNumber[1] = idBuffer[7];
  serialNumber[0] = idBuffer[6];
  switch(productId[2]){
    case 0x01:
      if(productId[3] == 0x86){
        this->scaleFactorDiffPressure = 60;
      }
      else{
        this->scaleFactorDiffPressure = 60;
      }
    break;
    case 0x0A:
      this->scaleFactorDiffPressure = 60;
    break;
    case 0x02:
      if(productId[3] == 0x86){
        this->scaleFactorDiffPressure = 240;
      }
      else{
        this->scaleFactorDiffPressure = 240;
      }
    break;
    case 0x0B:
      this->scaleFactorDiffPressure = 240;
    break;
    default:
      this->scaleFactorDiffPressure = 60; // as default
    break;
  }
  //Print values through Serial
  VERBOSE_PRINTLN();
  VERBOSE_PRINT("SDP Product Number = ");
  for(int i = 0; i < 4; i++){
    if(productId[i] < 0x10){
      VERBOSE_PRINT("0");
    }
    VERBOSE_PRINTHEX(productId[i]);
    VERBOSE_PRINT(" ");
  }
  VERBOSE_PRINTLN();
  VERBOSE_PRINT("Serial Number = ");
  for(int i = 0; i < 8; i++){
    if(serialNumber[i] < 0x10){
      VERBOSE_PRINT("0");
    }
    VERBOSE_PRINTHEX(serialNumber[i]);
    VERBOSE_PRINT(" ");
  }
  VERBOSE_PRINTLN();
  VERBOSE_DELAY(2000);
}
else{
  VERBOSE_PRINT("Error while receiving data, crc does not match");
  VERBOSE_DELAY(2000);
}

}
void SDP_Controller::getSerialNumber(void){
}
void SDP_Controller::begin(void){
  //this->getProductId();
  stopContinuousMeasurement();
  softReset();
  delay(20);
  getProductId();
  // set values fot scale factors
  //scaleFactorDiffPressure = this->getPresScaleFactor(productId);
}
/*
> Set the sensor to continuous measurement mode
 */
void SDP_Controller::startContinuousMeasurement(SdpTempComp tempComp,SdpAveraging averaging){
  switch(tempComp){
    case SDP_TEMPCOMP_MASS_FLOW:
      switch (averaging) {
        case SDP_AVERAGING_NONE:
        sendCommand(COMMAND_START_MEASURMENT_MF_NONE);
        break;
        case SDP_AVERAGING_TILL_READ:
        sendCommand(COMMAND_START_MEASURMENT_MF_AVERAGE);
        break;
      }
    break;
    case SDP_TEMPCOMP_DIFFERENTIAL_PRESSURE:
    switch (averaging) {
      case SDP_AVERAGING_NONE:
      sendCommand(COMMAND_START_MEASURMENT_DP_NONE);
      break;
      case SDP_AVERAGING_TILL_READ:
      sendCommand(COMMAND_START_MEASURMENT_DP_AVERAGE);
      break;
    }
    break;
  }
}

/* Stops continuous measurement mode, returns sensor to idle */
void SDP_Controller::stopContinuousMeasurement(void){
  sendCommand(COMMAND_STOP_CONTINOUS_MEASUREMENT);
}
/*
brief  > Get triggered temperature reading, works only if sensor in idle mode
return > float with triggered temperature value
 */
float SDP_Controller::getTemperatureTrigger(SdpTempComp tempComp, SdpClockStretching clkSt){
  switch(tempComp){
    case SDP_TEMPCOMP_MASS_FLOW:
      switch(clkSt){
        case SDP_CLKST_NONE:
        sendCommand(COMMAND_TRIGGER_MEASUREMENT_MF_NONE);
        break;
        case SDP_CLKST_ACTIVE:
        sendCommand(COMMAND_TRIGGER_MEASUREMENT_MF_CLK_STRETCHING);
        break;
      }
    break;
    case SDP_TEMPCOMP_DIFFERENTIAL_PRESSURE:
      switch(clkSt){
        case SDP_CLKST_NONE:
        sendCommand(COMMAND_TRIGGER_MEASUREMENT_DP_NONE);
        break;
        case SDP_CLKST_ACTIVE:
        sendCommand(COMMAND_TRIGGER_MEASUREMENT_DP_CLK_STRETCHING);
        break;
      }
    break;
  }
  delay(50);
    readSequence();
/* CHeck CRC fnc!
if(CheckCrc(this->dataBuffer, 3, this->dataBuffer[5] == true)){
    return BIU16(this->dataBuffer,3)/this->scaleFactorTemperature;
  }
  else{
    return 0;
  }*/
int16_t result = BIU16(this->dataBuffer,3);
return result/(float)200;
}

/*
Get triggered differential pressure reading, works only if sensor is in idle mode
 return -> float with triggered differential pressure value
 WARNING > MISSING FUNCTION TO CORRECT atmPressure with triggered measurement
 */
float SDP_Controller::getDiffPressureTrigger(SdpTempComp tempComp, SdpClockStretching clkSt){
  switch(tempComp){
    case SDP_TEMPCOMP_MASS_FLOW:
      switch(clkSt){
        case SDP_CLKST_NONE:
        sendCommand(COMMAND_TRIGGER_MEASUREMENT_MF_NONE);
        break;
        case SDP_CLKST_ACTIVE:
        sendCommand(COMMAND_TRIGGER_MEASUREMENT_MF_CLK_STRETCHING);
        break;
      }
    break;
    case SDP_TEMPCOMP_DIFFERENTIAL_PRESSURE:
      switch(clkSt){
        case SDP_CLKST_NONE:
        sendCommand(COMMAND_TRIGGER_MEASUREMENT_DP_NONE);
        break;
        case SDP_CLKST_ACTIVE:
        sendCommand(COMMAND_TRIGGER_MEASUREMENT_DP_CLK_STRETCHING);
        break;
      }
    break;
  }
  delay(50);
  readSequence();/*
  Serial.println(" ");
  Serial.print(this->dataBuffer[0], HEX);
  Serial.print(this->dataBuffer[1], HEX);
  Serial.print(this->dataBuffer[2], HEX);
  Serial.println();*/
int16_t result = BIU16(this->dataBuffer,0);
return result/(float)60;
  /*if(CheckCrc(rxBuffer, 0, rxBuffer[3])){
    return BIU16(rxBuffer,0)/this->scaleFactorDiffPressure;
  }
  else{
    return 0;
    DEBUG_PRINT("getDiffPressureTrigger(tempComp,averaging) failed");
  }*/
}
/*
Get continuous mode reading diffPressure, needs continous mode
eturn > float with continuous differential pressure value
 */
float SDP_Controller::getDiffPressure(void){
  //*** get measurement mode from startContinuousMeasurement fcn
  if(readSequence() == 0){
  return (int16_t)BIU16(this->dataBuffer,0)/(float)60;
  }
  else{
    return 0;
    DEBUG_PRINT("getDiffPressure() failed");
  }
}
/*
brief  > Get continuous mode reading diffPressure
param  > Atmosferic pressure to compensate measurement (float)
return > float with compensated continuous differential pressure value
 */
float SDP_Controller::getDiffPressure(float atmPressure){
  if(readSequence() == 0){
  return (int16_t)BIU16(this->dataBuffer,0) * (float)966 / (atmPressure * (float)60);
  }
  else{
    return 0;
    DEBUG_PRINT("getDiffPressure(atmPressure) failed");
  }
}
/*
brief  >Set to get continuous mode reading diffPressure
param  > Atmosferic pressure to compensate measurement (double)
return > float with compensated continuous differential pressure value
 */
float SDP_Controller::getDiffPressure(double atmPressure){
  if(readSequence()== 0){
  return (int16_t)BIU16(this->dataBuffer,0) * (float)966 / (atmPressure * (float)60);
  }
  else{
    return 0;
  }
}
/*
brief  > Get continuous mode reading of temperature, needs continuous mode
return > Float with continuous temperature value
notes  > needs to have called getDiffPressure first
 */
float SDP_Controller::getTemperature(void){
  //*** get measurement mode from startContinuousMeasurement fcn.
  return (int16_t)BIU16(this->dataBuffer,3)/(float)200;
}

/*
brief  > set to get only continuous mode reading of temperature, needs continuous mode
param  > none
return > float with continuous temperature value
 */
float SDP_Controller::getOnlyTemperature(void){
  /* get measurement mode from startContinuousMeasurement fcn. */
  readSequence();
  return (int16_t)BIU16(this->dataBuffer,3)/(float)200;
}
void SDP_Controller::softReset(void){
  sendCommand(COMMAND_SOFT_RESET); //Soft reset sent to the sensor
}
void SDP_Controller::enterSleep(void){
  sendCommand(COMMAND_ENTER_SLEEP_MODE); //Set the sensor sleep mode lower down the energy consumption
}
void SDP_Controller::exitSleep(void){ //See the product manual
}

/* PRIVATE FUNCTIONS */
bool SDP_Controller::CheckCrc(uint8_t data[], uint8_t posInit, uint8_t checksum){
  uint8_t crc = 0xFF; //

  // calculates 8-Bit checksum with given polynomial 0x31 (x^8 + x^5 + x^4 + 1)
  for(int i = posInit; i < posInit + 2; i++) {
    crc ^= (data[i]);
    for(uint8_t bit = 8; bit > 0; --bit) {
      if(crc & 0x80) {crc = (crc << 1) ^ 0x31;}
      else           {crc = (crc << 1);}
    }
  }
  if(crc == checksum){
    return true;
  }
  else{
    DEBUG_PRINT("Crc does not match");
    return false;
  }
}
/*
brief  > checks all the crcs of the sequence received
param  > sequence vector, length of
return > bool, true if correct
 */
bool SDP_Controller::checkCrcRoutine(uint8_t idBuffer[]){ 
  bool result = true;
  for(uint8_t i = 0; i<(sizeof(idBuffer) - 2); i = i+3){
    if(!CheckCrc(idBuffer,  i, idBuffer[i+2])){
      result = false;
    }
  }
  return result;
}
/*
brief  > hal function for sending the i2c commands
param  > Command type for abstraction
return > none
 */
Error SDP_Controller::sendCommand(Command cmd){
  Wire.beginTransmission(DEFAULT_SDP_ADDRESS);
  Wire.write(B16TO8_1(cmd));
  Wire.write(B16TO8_2(cmd));
  uint8_t errorByte = Wire.endTransmission();
  Error error = setError(errorByte);
  //Error error = setError(Wire.endTransmission());
  return error;
}

Error SDP_Controller::setError(uint8_t errorByte){
  Error error;
    switch(errorByte){
      case 0:
        error = ERROR_NONE;
        break;
      case 1:
        error = ERROR_DATA;
        DEBUG_PRINT("ERROR_DATA");
        break;
      case 2:
        error = ERROR_NACK_ADDRESS;
        DEBUG_PRINT("ERROR_NACK_ADDRESS");
        break;
      case 3:
        error = ERROR_NACK_DATA;
        DEBUG_PRINT("ERROR_NACK_DATA");
        break;
      case 4:
        error = ERROR_OTHER;
        DEBUG_PRINT("ERROR_OTHER");
        break;
      default:
        error = ERROR_OTHER;
        break;
    }
  return error;
}
/*
brief  > Set to reads i2c rx sequence and saves it in dataBuffer
*/
Error SDP_Controller::readSequence(){
  uint8_t rxByteCount = 0;
  Wire.requestFrom((uint8_t)DEFAULT_SDP_ADDRESS, (uint8_t) 9);
  while(Wire.available()){
    this->dataBuffer[rxByteCount] = Wire.read();
    rxByteCount++;
  }
  if(rxByteCount != 9 || !checkCrcRoutine(this->dataBuffer)){
    DEBUG_PRINT("Error receiving i2c sequence");
    return ERROR_RECEIVE;
  }
  else{
    return ERROR_NONE;
  }
}
Error SDP_Controller::readIdSequence(){
  uint8_t rxByteCount = 0;
  Wire.requestFrom((uint8_t)DEFAULT_SDP_ADDRESS, (uint8_t) 18); //Correct number of bytes to read
  while(Wire.available()){
    this->idBuffer[rxByteCount] = Wire.read();
    rxByteCount++;
  }

  if(rxByteCount != 18 || !checkCrcRoutine(this->idBuffer)){
    //Serial.print("Error receiving!!!");
    return ERROR_RECEIVE;
  }
  else{
    return ERROR_NONE;
  }
}
/*
brief  > Set to 0btain the correct scale factor for the model of sensor
param  > none
return > float with the scaleFactor
 */
float SDP_Controller::getPresScaleFactor(uint8_t productId[]){ 
  switch (productId[3]) {
    case SDP800_500Pa:
    return 60;
    break;
    case SDP810_500Pa:
    return 60;
    break;
    case SDP800_125Pa:
    return 240;
    break;
    case SDP810_125Pa:
    return 240;
    break;
    case SDP31:
    return 60;
    break;
    case SDP32:
    return 240;
    break;
  }
}