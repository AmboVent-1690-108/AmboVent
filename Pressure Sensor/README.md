## General comments on the pressure sensor

_Please note that this sensor requires VDD of 3.3V and pull up to signal on the **SAME** VDD input, so using the current board schem could **damage** the sensor_

# Presure sensor and board
---------------------------------

We used the "SparkFun Pressure Sensor Breakout - MS5803-14BA"
The item can be found at the following link for $60US:
https://www.sparkfun.com/products/12909
It also can be found at other suppliers usually for the same price.

It is based of the TE MS5803-14BA sensor which can be bought
separately for cheaper price but usually in quantities of hundreds.
https://www.futureelectronics.com/p/analog--sensors--pressure/ms580314ba01-00-te-sensor-solutions-5058846
https://www.mouser.com/ProductDetail/Measurement-Specialties/MS580314BA01-00?qs=sGAEpiMZZMueJGT%2F0PGc4wvItXGMkWfD%252B0JwHIbyHlk%3D
If one uses the basic TE sensor a board should be designed with
few capacitors and resistors.

In case one want to use a different pressure sensor, we
recommend the following:
1. Pressure range is 100mbar (100cm H2O)
2. Digital communication - I2C preferred (analog communication 
will require high frequency averaging)

In case one prefer to use differential pressure sensor it will
improve reliability but will require a slightly change in the
arduino code.


Presure sensor box
---------------------------------

The box and cover are made from liquid ABS, using SLA technology
with resolution 0.05 to allow sealed box.

It is recommended to use flexible silicone wires to reduce stress
when mounting the pressure sensor in the box. 

Use a sealing silicone on the wires and around the box cover to
ensure complete sealed box
