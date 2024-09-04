#!/bin/bash

export RPI=3

# Build dependencies and MiniDexed
# ./build.sh
cd src
make -j
cd ..
cp ./src/kernel*.img ./kernels/

# Get Raspberry Pi boot files
# cd ./circle-stdlib/libs/circle/boot
# make
# if [ "${RPI}" -gt 2 ]
# then
# 	make armstub64
# fi
# cd -

# Make zip that contains Raspberry Pi 4 boot files. The contents can be copied to a FAT32 formatted partition on a microSD card
# cd sdcard
# ../getsysex.sh
# cd ..
# cp -r ./circle-stdlib/libs/circle/boot/* sdcard
# rm -rf sdcard/config*.txt sdcard/README sdcard/Makefile sdcard/armstub sdcard/COPYING.linux
# cp ./src/config.txt ./src/minidexed.ini ./src/*img ./src/performance.ini sdcard/
# echo "usbspeed=full" > sdcard/cmdline.txt
# cd sdcard
# cp ../kernels/* . || true
# zip -r ../MiniDexed_$GITHUB_RUN_NUMBER_$(date +%Y-%m-%d).zip *
# cd -
