AmboVent Electronic Design
--------------------------
This directory contains the files needed to produce and/or change the electronic control board of AmboVent.

The controller consist of a single PCB, with Arduino Nano mounted on it and a few other components. The board contains header connectors for varios system components:
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
