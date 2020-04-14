# AmboVent  1690.108

[![Travis Status][S]][T]

    Automatic, Controlled Resuscitator Device
    Emergency ventilation alternative system
    Global Partnership for the Greater good
    Leading Open Source Code Mentality Initiative


## Emergency ventilation initiative coming out of Israel

## The device was not tested yet for its performance or safety; it is NOT approved for clinical use

### Who is behind it?  
**Lead by the CTO & innovation leader of the Israeli Air-Force 108 Electronic Depot and backed by a large community of innovators behind him, to include:**

1. 40 Professional Volunteers
1. Magen David Adom (Israel’s national EMS)
1. Physicians from leading Israeli hospitals such as Tel Aviv Sourasky and Hadassah JLM as well as other medical centers, Engineers, FIRST Israel mentors and students
1. The Haifa Technological Center Rafael
1. Israel Aerospace Industries
1. IAF Unit 108
1. The garage program by Microsoft Israel
1. IDC Herzeliya
1. and others...

## [AmboVent Website](https://1nn0v8ter.rocks/AmboVent-1690-108)

## [BOM list](https://docs.google.com/spreadsheets/d/1yqxRhruZpR-hO4cpbX6UkSeXTlXk1JLd_4aYDwuWj4k/edit?usp=sharing)

# AmboVent's Standard Copyright and Disclaimer Notice:

*Copyright ©2020. THE AMBOVENT GROUP FROM ISRAEL herby declares:  No Rights Reserved. Anyone in the world have Permission to use, copy, modify, and distribute this software and its documentation for educational, research, for profit, business and not-for-profit purposes, without fee and without a signed licensing agreement, all is hereby granted, provided that the intention of the user is to use this code and documentation to save human lives  anywhere in the world. For any question, contact dreliram@gmail.com.*

***IN NO EVENT SHALL THE AMBOVENT GROUP FROM ISRAEL, BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS, ARISING OUT OF THE USE OF THESE SPECS, LIST OF MATERIALS, ELECTRONIC DESIGNS , SOFTWARE CODE AND ANY DOCUMENTATION, EVEN IF THE AMBOVENT GROUP FROM ISRAEL HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
THE AMBOVENT GROUP FROM ISRAEL DISCLAIMS ANY WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE AND ACCOMPANYING DOCUMENTATION, SPECS, LIST OF MATERIALS, ELECTRONIC DESIGNS, SOFTWARE CODE AND ANY DOCUMENTATION IF PROVIDED HEREUNDER IS PROVIDED "AS IS". THE AMBOVENT GROUP FROM ISRAEL HAS NO CLAIM THAT IT IS COMPLETED OR FUNCTIONAL AND HAS NO OBLIGATION TO PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.***


# Setup

## Software

1. (Optional) Eclipse project for Arduino source code _editing_ (NOT building):
    1. A general setup PDF guide can be found in the [eRCaGuy_dotfiles](https://github.com/ElectricRCAircraftGuy/eRCaGuy_dotfiles) project [here](https://github.com/ElectricRCAircraftGuy/eRCaGuy_dotfiles/blob/master/eclipse/Eclipse%20setup%20instructions%20on%20a%20new%20Linux%20(or%20other%20OS)%20computer.pdf). See also:
    1. [3-Software/Arduino/readme.md](3-Software/Arduino/readme.md).
    1. [3-Software/Arduino/arduino_core/readme.md](3-Software/Arduino/arduino_core/readme.md)
    1. [3-Software/Arduino/.clang-format](3-Software/Arduino/.clang-format) - there are some nice descriptions & info in here on Eclipse & Sublime Text 3 editors, as well as setting your Arduino IDE to use 4 spaces instead of 2 for tabs.
1. `clang-format` automatic code format tool. Please run this on any PR before submitting!
    1. Linux (tested--works): 
        1. Install: 

                # Install it:
                sudo apt update
                sudo apt install clang-format

                # Ensure you have version 6 (the default for Ubuntu 18), for compatibility with 
                # the .clang-format file we are using; sample output: 
                # `clang-format version 6.0.0-1ubuntu2 (tags/RELEASE_600/final)`
                clang-format --version
        
        1. Use it: 
            1. Commit your changes so everything is backed up.
            1. Then run the clang formatter:

                    ./run_clang-format.sh

            1. Then review and commit these changes, if any.
            1. Now do a Pull Request.

    1. Windows:
        1. Install:
            1. Download and install [Git for Windows](https://git-scm.com/download/win)
            1. Download and install LLVM (which includes `clang-format`) version **6.0.1** (for compatibility--see above) [for Windows, here](https://releases.llvm.org/download.html). Here's a [direct link](https://releases.llvm.org/6.0.1/LLVM-6.0.1-win64.exe) if you like for the 64-bit version.
        1. Use it:
            1. Using the git bash terminal that comes with git for Windows, run the script:

                    ./run_clang-format.sh

            1. Make sure to do this before making any Pull Request (see Linux "Use it" instructions just above as well).




[S]: https://travis-ci.org/labeneator/AmboVent.svg?branch=master
[T]: https://travis-ci.org/github/labeneator/AmboVent