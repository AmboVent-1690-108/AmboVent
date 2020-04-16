Arduino library for AS5048B from AMS
==============

AS5048B is a 14-bit magnetic rotary position sensor with digital angle (I2C) and PWM output.
This library deals only with the I2C channel.

[AS5048B's](http://www.ams.com/eng/Products/Position-Sensors/Magnetic-Rotary-Position-Sensors/AS5048B) AMS page.

	v1.0 - First release
	v1.0.1 - Typo to allow compiling on Codebender.cc (Math.h vs math.h) + Wind vane example modification to comply with the Timer.h lib used by them
	v1.0.2 - ams_as5048b.cpp - fix setZeroReg() issue raised by @MechatronicsWorkman
	v1.0.3 - Small bug fix and improvement by @DavidHowlett
	v1.0.4 - Implemented OTP register burning by @brentyi
	v1.0.5 - Optional parameters fix


## Features ##
- Manage zero position
- Counts CW or CCW
- Reads Auto Gain & Diagnostics registers
- Reads 14 bits magnitude
- Reads 14 bits angle with various units output (raw, turn, degree, radian, grade, minute of arc, second of arc, Nato mil, Russian mil, Swedish mil)
- Computes an angular exponential moving average
- Reads exponential moving average angle and outputting with various units
- Resets Exp moving Avg
- OTP setting
- OTP programming sequence

## Code examples ##
- Single angle reading, outputs 2 units
- Angular exponential moving average reading, outputs read angle and average
- Wind vane, outputs azimuth and compass direction - This one as a special #define for Codebender.cc support
- Dial reading for X-Plane
- Slave address programming

## Not available yet features ##
- PWM reading
- Debug

## Testing ##
- Tested against AS5048B's official [adapter board](http://www.ams.com/eng/Support/Demoboards/Position-Sensors/Rotary-Magnetic-Position-Sensors/AS5048B-Adapterboard)
- Tested on Arduino Mega with Arduino IDE 1.0.5 && Codebender.cc
- Tested on Arduino Uno with Arduino IDE 1.6.9
- Tested on Arduino Nano with Arduino IDE 1.6.9
- Tested on Teensy++ 2.0 with Arduino IDE 1.6.9
- Please comment about other devices
