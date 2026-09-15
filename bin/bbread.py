#!/usr/bin/env python3
import signal
from datetime import datetime, timezone

import serial

port = "/dev/ttyUSB1"
# speed = 230400
speed = 500000

now = datetime.now(timezone.utc).astimezone()  # current date and time, local timezone
filename = now.strftime("LOG_%y%m%d_%H%M%S.BFL")
stop_processing = False


def handler(_signum, _frame):
    global stop_processing
    stop_processing = True
    print("Exiting")


signal.signal(signal.SIGINT, handler)
print("Press CTRL^C to stop")

with serial.Serial(
    port=port,
    baudrate=speed,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=0.2,
) as ser:
    print("Serial opened for read:", port, speed)
    with open(filename, "wb") as f:
        print("File opened for write:", filename)
        while not stop_processing:
            f.write(ser.read())
