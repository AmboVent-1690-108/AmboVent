Inside this folder should be a symbolic link to your arduino installation folder. This way, an Eclipse project can have full access to all of the Arduino source code to allow jumping to the core implementations of Arduino functions and definitions while exploring the code. 

## References:
1. How to make symbolic links in Windows: 
   https://www.howtogeek.com/howto/16226/complete-guide-to-symbolic-links-symlinks-on-windows-or-linux/

## How to make the symbolic link to the core Arduino source code:

**1) To create a symbolic link on Mac or Linux**, the command looks like this. This will create a symbolic link folder called "arduino" here: 

    ln -s "/path/to/arduino_installation_folder" arduino

Example: 

    cd /path/to/here
    ln -s "/home/gabriel/Downloads/Install_files/Arduino/arduino-1.8.8" arduino


**2) To create a symbolic link on Windows**, the command looks like this:

    mklink /D arduino "C:\Users\gabriel\Downloads\Install_files\Arduino\arduino-1.8.8" 

