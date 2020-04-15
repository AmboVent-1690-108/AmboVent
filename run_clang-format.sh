#!/bin/bash

# This is a convenience script at the root of the repo just to make it easier to call the main
# run_clang-format.sh script.
#
# For info on how to use clang-format, see "git & Linux cmds, help, tips & tricks - Gabriel.txt"
# doc in https://github.com/ElectricRCAircraftGuy/eRCaGuy_dotfiles. Direct link:
# https://github.com/ElectricRCAircraftGuy/eRCaGuy_dotfiles/blob/master/git%20%26%20Linux%20cmds%2C%20help%2C%20tips%20%26%20tricks%20-%20Gabriel.txt
#
# Other References:
# 1. Main documentation, setup, instructions, etc! https://clang.llvm.org/docs/ClangFormat.html
# 2. Download the Windows & other installers/executables: https://llvm.org/builds/
# 3. Clang-Format Style Options: https://clang.llvm.org/docs/ClangFormatStyleOptions.html

# Obtain path to this script.
# See my own ans here: https://stackoverflow.com/questions/59895/how-to-get-the-source-directory-of-a-bash-script-from-within-the-script-itself/60157372#60157372
THIS_PATH="$(realpath $0)"
THIS_DIR="$(dirname "$THIS_PATH")"
# echo "THIS_DIR = \"$THIS_DIR\"" # for debugging 

# Call the main run_clang-format script. 
"$THIS_DIR/3-Software/Arduino/run_clang-format.sh" 
