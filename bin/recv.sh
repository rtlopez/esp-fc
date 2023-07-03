#!/bin/bash -ex

PORT="/dev/ttyUSB1"
SPEED="115200"
SPEED="230400"
#SPEED="250000"
#SPEED="500000"

LOGFILE="LOG_$(date +%y%m%d%H%M).BFL"

stty -F $PORT $SPEED
cat < $PORT > $LOGFILE

