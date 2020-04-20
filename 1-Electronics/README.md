AmboVent Electronic Design
--------------------------
## This directory contains the files needed to produce and/or change the electronic control board of AmboVent.

### The controller consist of a single PCB, with Arduino Nano mounted on it and a few other components. The board contains header connectors for varios system components:
- Front panel controls (pushbuttons, potentiometers, LEDs)
- Power input
- PWM control to the main motor
- Arm location feedback (potentiometer)
- I2C pressure sensor
 
All connectors have 0.1" spacing, and can accept either male or female headers. The power
input connector has 5mm spacing and it is designed for screw terminal connector type, but
direct wire connection is also feasible.

The PCB is designed with single layer, quick CNC fabrication in mind. It can be milled
with 1/32 mill. In this case, assembly will require a few wire bridges. If traditional, 
two layer fabrication method is used, a top layer can be used in place of the wire bridges.

#### SW1 – D2 - test button (used to initiate calibration)
#### SW2 – D4 - toggle breath on/off
#### SW3 – D5 - reset alarm button
#### Pot 1 – A2 – sets the range of motion as percentage from full range as set in calibration
#### Pot 2 – A3 – sets the respiration rate from 6 to 24 per minute
#### Pot 3 – A6 – sets the inspirium pressure from 30 to 70 cm H2O
#### J3 – connects to the position sensor potentiometer in the arm
#### J4 – connects to the motor controller PWM input (usually don’t connect the 5V)
#### J5 – connector to pressure sensor – if you use a 3.3V pressure sensor – supply 3.3V from the Arduino instead of the 5V

**Assemble the potentiometer for arm position sense such that when the arm moves down (presses the
Ambu more) – the voltage goes up. Range 0-5 volts
Assemble the user interface potentiometers such that during clockwise motion the voltage goes up.
Range 0-5 volts
Motor polarity shall be set such that PWM>50% to the controller results in the arm pressing the Ambu
(moving down).
The SW calibration procedure enable easy verification of the motor and position feedback polarity
(see calibration video for details)**

_Please note that the current sensor is **NOT** needed if you are using pressure sensor, it's possible to use it to monitor and limit the motor current while using the pressure sensor but again, not mandatory._
