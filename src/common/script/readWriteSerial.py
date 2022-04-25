#!/usr/bin/env python

import serial
import time 
device = serial.Serial('/dev/SBF')
# device.baudrate(9600)
time.sleep(6)
device.write('A')

print("message sent")
time.sleep(0.1)
response = device.read()
print(response)