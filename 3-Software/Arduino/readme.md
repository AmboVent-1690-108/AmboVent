## Description of files & folders:

## Main folders:
1. arduino_core = contains a symbolic link called "arduino" to the arduino installation directory so that your Eclipse project, for instance, can index it and allow you to jump to it while exploring the code
2. Libraries = required Arduino libraries you need to build this project
3. ventilation_machine = the main source code for this project

## Other files & folders:
1. .settings = Eclipse folder for this project
2. .cproject = Eclipse project file
3. .project = Eclipse project file
4. readme.md = this file

**Note that if you're new to Eclipse, a general setup PDF guide can be found in the [eRCaGuy_dotfiles](https://github.com/ElectricRCAircraftGuy/eRCaGuy_dotfiles) project [here](https://github.com/ElectricRCAircraftGuy/eRCaGuy_dotfiles/blob/master/eclipse/Eclipse%20setup%20instructions%20on%20a%20new%20Linux%20(or%20other%20OS)%20computer.pdf).**

When setting up an Eclipse project for Arduino, just set it up as a C/C++ project, then tell Eclipse that `*.ino` files are a type of C++ Source File (so that it will index them and open them up in its C/C++ editor with proper syntax highlighting) by following these instructions here: https://stackoverflow.com/questions/33474629/is-ecplise-cdts-indexer-limited-to-the-common-filetypes-for-sources-and-headers/33520998#33520998. In short: Window --> Preferences --> C/C++ --> File Types --> click "New..." --> type in "*.ino" as the pattern, and set the "Type" to "C++ Source File" --> click "OK" --> "Apply and Close". Eclipse will now automatically re-index the whole project, indexing all *.ino files as C++ source files! *These instructions have now been added to my Eclipse PDF doc above!*

