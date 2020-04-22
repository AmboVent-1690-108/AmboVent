<!-- Note that this is a markdown comment. It won't show up in your text. You don't have to delete
these. -->
## Short description (1~4 lines):
<!-- What problems does this solve? How? Use the longer details section below if needed. -->


## Extra Details:
<!-- (Optional) extra details, continued from above;
Delete this section if you don't need it. -->


## Checklist:
<!-- You may delete this whole section if your PR is not modifying any software/code. -->
<!-- Add an X inside the square brackets below when you've completed each item. This PR cannot
merge until you have completed all items. -->

- [ ] I have run the code formatter with `./run_clang-format.sh` (see main 
[README.md](https://github.com/AmboVent-1690-108/AmboVent#software) for details on how)
- [ ] I have built the code in the Arduino IDE (or with the command-line tool) and pasted the 
build output below
- [ ] If I have touched *any* lines of code in the software, other than comments, I have incremented
the version number string at the top of "ventilation_machine.ino" and updated the Software Changelog
in the main AmboVent readme.

**Arduino IDE build output:**  

<!-- YYYYMMDD-HHMMhrs is the date and time you ran the build command; ex: 20200420-0049hrs means 20
April 2020 at 0049 hrs (12:49AM--just after midnight). Also be sure to include at least the first 7
chars of your git hash for the commit you had checked out when you performed the build. Lastly,
specify your Arduino IDE version number and board. The board should be Nano unless you know
differently. -->
YYYYMMDD-HHMMhrs (git hash <7+ char git hash>) (IDE ?.?.??) (board: Arduino Nano)

    Sketch uses 99999 bytes (99%) of program storage space. Maximum is 32256 bytes.
    Global variables use 9999 bytes (99%) of dynamic memory, leaving 9999 bytes for local variables. Maximum is 2048 bytes.

<!-- Example:
20200420-0049hrs (git hash 493985f; branch fix_formatting) (IDE 1.8.12) (Arduino Nano) - PR #46: https://github.com/AmboVent-1690-108/AmboVent/pull/46

    Sketch uses 18176 bytes (56%) of program storage space. Maximum is 32256 bytes.
    Global variables use 1046 bytes (51%) of dynamic memory, leaving 1002 bytes for local variables. Maximum is 2048 bytes.
-->

