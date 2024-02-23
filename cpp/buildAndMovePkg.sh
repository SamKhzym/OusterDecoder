#!/bin/bash

# -- AUTHOR: Sam Khzym --

# -- DESCRIPTION --
# buildAndMovePkg -- call this in cmd line with 1st arg being the name of the package you want to create. 
# Optional -c flag will clear your build directory before recompile. 
# Might need to run with sudo.

# -- EXAMPLE USAGE --
#"./buildAndMovePkg -c" will clear my build dir and copy my .pck file to RTMAPS_INSTALL/packages/sam_tuts

if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    BUILD_TARGET=Linux64 #Linux64 or Win64
elif [[ "$OSTYPE" == "msys" ]]; then
    BUILD_TARGET=Win64 #Linux64 or Win64
fi

#cmd line arg check
if ! [[ $# == 0 || $# == 1 && $1 == -c ]]
then
    echo Incorrect number of cmd line args. Only optional -c flag to clear build folder required. Terminating.
    exit
fi

if [ $# == 1 ]
then
    if [ -d build ]; then rm -Rf build; fi
fi

#cmake build
export DEBUG=1

mkdir -p build
cd build || exit
cmake -DCMAKE_BUILD_TYPE=Debug ..
cmake --build . --config Debug -j

#move generated package to packages folder under rtmaps installation
pckName=$(find ./*.pck)
pckName=${pckName#./}
pckNameWithTarget=${pckName%.*}_${BUILD_TARGET}.${pckName##*.}

#create a bin directory if it doesn't already exist and put pck file in there
mkdir -p ../bin
scp "$pckName" ./../bin/"$pckNameWithTarget"