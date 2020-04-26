# Library for Sensirion SDP 3x and 8xx differential pressure sensors

This is a library for the digital variants of Sensirion SDP sensors. This library has been tested with the SDP810 but should work with the following sensors:
- SDP31 (500Pa)
- SDP32 (125Pa)
- SDP800 - 500Pa    
- SDP800 - 125Pa
- SDP810 - 500Pa
- SDP810 - 125Pa


# About the sensors
The digital differential pressure sensors of the SDP8xx and SDP 3x series provide a digital I²C output, excellent long-term stability and measure with high sensitivity and accuracy even at low differential pressures (10 Pa).

The base of these sensors is a miniature heating element and two temperature sensors. Difference of pressures will cause a small air flow through a component and it will cause a temperature difference in these built-in sensors. Temperature difference is directly depending on the amount (weight) of air flowing through a sensor and it naturally depends just on a pressure difference. A great advantage against membrane pressure sensors is a long-term high accuracy even at measuring of small pressure and maintaining of accuracy of a “zero point”.
# Compatibility
This library works as is with Arduino and Teensy boards. To make this library compatible with other MCUs the functions sdp.sendCommand() and sdp.readSequence() would have to be adapted, the rest of the library is hardware agnostic. These functions work as a sort of hardware abstraction layer.

# How to use
Declaration of object of the SDPclass:
```sh
SDPclass sdp;
```
From this point onwards the object used will have the name **sdp**.

**sdp.getAddress();** - Searches the i2c address of the sensor, other sensors connected to the same port will be shown as well.
- param - none
- return -none

**sdp.getProductId();** - Shows in serial console the Product ID, saves the result in a private variable to select the adequate scaleFactor for the sensor
- param - none
- return - none

**sdp.begin();** -Init routine of the sensor, reads productId to select appropriate scaleFactor. Sets the sensor to idle mode stopping possible previous mode as after a reset without powering off the sensor it keeps the previous configuration.
- param - none
- return - none

**sdp.startContinuousMeasurement(tempComp, averaging);** - Set the sensor to continuous measurement mode with the selected temperature compensation and averaging modes.
- param -
  - tempComp = SDP_TEMPCOMP_MASS_FLOW, SDP_TEMPCOMP_DIFFERENTIAL_PRESSURE;
  - averaging = SDP_AVERAGING_NONE, SDP_AVERAGING_TILL_READ
- return - none

**sdp.stopContinuousMeasurement();** - Stops continuous measurement mode, returns sensor to idle
- param - none
- return - none

**spd.getDiffPressure();** - Get continuous mode reading differential pressure, it needs continous mode to have been started previously (sdp.startContinuousMeasurement(tempComp, averaging))
- param - none
- return - float with the differential pressure value

**spd.getDiffPressure(atmPressure);** - Get continuous mode reading differential pressure, it needs continous mode to have been started previously (sdp.startContinuousMeasurement(tempComp, averaging))
- param - float with the current atmosferic pressure for correction of the differential pressure value in mbar
- return - float with the differential pressure value

**sdp.getTemperature();** - Get continuous mode reading of temperature, shows last value obtained with sdp.getDiffPressure() or sdp.getDiffPressure(atmPressure);
- param - none
- return - float with the temperature value in ºC

**sdp.getOnlyTemperature();** - Get continuous mode reading of temperature, it does not need the previous calling of a differential pressure reading.
- param - none
- return - none

**sdp.getTemperatureTrigger(tempComp, clkst);** - Get triggered temperature reading, works only if sensor in idle mode
- param -
  - tempComp = SDP_TEMPCOMP_MASS_FLOW, SDP_TEMPCOMP_DIFFERENTIAL_PRESSURE
  - clkst = SDP_CLKST_NONE, SDP_CLKST_ACTIVE
- return - float with the temperature value in ºC

**sdp.getDiffPressureTrigger((tempComp, clkst);** - Get triggered differential pressure reading, works only if sensor is in idle mode
- param -
  - tempComp = SDP_TEMPCOMP_MASS_FLOW, SDP_TEMPCOMP_DIFFERENTIAL_PRESSURE
  - clkst = SDP_CLKST_NONE, SDP_CLKST_ACTIVE
- return - float with the differential pressure value in Pa


**sdp.softReset();** - Executes a soft reset of the sensor, leaving it in idle mode
- param - none
- return - none


**sdp.enterSleep();** - Sets the sensor in sleep mode to minimize energy consumption
- param - none
- return - none


**sdp.exitSleep();** - Gets the sensor out of sleep mode
- param - none
- return - none

# Debug and Verbose modes
Through uncommenting the *#defines* for DEBUG and VERBOSE we can have serial output for inner working and errors.

**();** -
- param -
- return -
