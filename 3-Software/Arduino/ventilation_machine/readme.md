
## Build size notes:
(Newest on TOP; date format is YYYYMMDD)

20200420-0049hrs (git hash 493985f; branch fix_formatting) (IDE 1.8.12) (Arduino Nano) - PR #46: https://github.com/AmboVent-1690-108/AmboVent/pull/46

    Sketch uses 18176 bytes (56%) of program storage space. Maximum is 32256 bytes.
    Global variables use 1046 bytes (51%) of dynamic memory, leaving 1002 bytes for local variables. Maximum is 2048 bytes.

20200419-2312hrs (git hash 078871e05ddd5890747f49222d46a7b30fd7b023; branch master) (IDE 1.8.12) (Arduino Nano) - no changes made yet to the code:

    Sketch uses 18174 bytes (56%) of program storage space. Maximum is 32256 bytes.
    Global variables use 1046 bytes (51%) of dynamic memory, leaving 1002 bytes for local variables. Maximum is 2048 bytes.


20200411-2217hrs (IDE 1.8.8) (Arduino Nano) - no changes made yet to the code:

    Sketch uses 18280 bytes (59%) of program storage space. Maximum is 30720 bytes.
    Global variables use 1882 bytes (91%) of dynamic memory, leaving 166 bytes for local variables. Maximum is 2048 bytes.
    Low memory available, stability problems may occur.



## Bluetooth module on D0-D1 issues:
The main issue is with pins 0 and 1.

Many pins have multiple functions assigned to them, such as A4 and A5 are also the I2C pins, pins 10/11/12/13 are also the SPI pins, etc. But pins 0 and 1 are the only multi-function pins that actually have something attached to them on the board.

All the other pins, while being multi-function, have their function defined basically by what you attach to them. Since the pins 0 and 1 are already attached on the board to the USB interface chip their usage possibilities are somewhat more limited.

As soon as you enable Serial in your sketch (Serial.begin()) those two pins can no longer be reliably used for digital IO.
That means that you can either use the hardware serial port or you can use the pins for digital IO, but not both.

Devices connected to pins 0 and 1 can interfere with the serial connection
That includes uploading of sketches. You have serial data coming in from the PC while the same pins are being affected by other things connected to them. The most common one is things like GPS modules which connect to those pins and constantly send serial data. That serial data will conflict with the PC's serial data and neither will arrive right.

How do you upload sketches if the serial is being interfered with by your sketch? How do you "reset" the board?
You don't. It's not your sketch that's interfering with the serial, it's what is physically connected to those two IO pins that are interfering. Just disconnect those two pins from whatever is connected and you will be able to upload sketches again. Many shields now are starting to add a small switch on them to disconnect those pins so you can upload sketches without unplugging the shield. You can also use the same trick with other devices you wire up to those pins - add a double-pole-single-throw or double-pole-double-throw (and not use one position) to easily isolate both pins from the rest of your hardware when you need to.

Does this affect all Arduino boards?
No, only those that use a USB to Serial bridge chip - that's things like the Uno, Due, Mega, etc. Boards that have a direct USB connection don't use the TX and RX pins for uploading sketches - they use the dedicated USB D+ and D- pins. That's boards like the Leonardo, some of the smaller (mini? micro? I forget which) boards, etc.
