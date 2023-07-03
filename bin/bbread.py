#!/usr/bin/env python3
import serial
import signal
from datetime import datetime

port = "/dev/ttyUSB1"
#speed = 230400
speed = 500000

now = datetime.now() # current date and time
filename = now.strftime("LOG_%y%m%d_%H%M%S.BFL")

f = open(filename, "wb")
print("File opened for write:", filename)

ser = serial.Serial(
    port=port,
    baudrate=speed,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=0.2
)
print("Serial opened for read:", port, speed)

stopProcessing = False

def handler(signum, frame):
    global stopProcessing
    stopProcessing = True
    print("Exiting")

signal.signal(signal.SIGINT, handler)
print("Press CTRL^C to stop")

while True:
    if stopProcessing: break
    x = ser.read()
    f.write(x)

ser.close()
f.close()
