#!/bin/env python2

import serial
import time
import uinput
ser = serial.Serial('/dev/ttyS31', 9600)
events = (uinput.BTN_JOYSTICK, uinput.ABS_X + (0, 255, 0, 0))
device = uinput.Device(events)
device.emit(uinput.ABS_X, 128, syn=False)
while True:
    value = ser.readline()
    valuecorrect = value.strip()
    valuecorrect = int(valuecorrect)/4
    print valuecorrect
    device.emit(uinput.ABS_X, int(valuecorrect))
