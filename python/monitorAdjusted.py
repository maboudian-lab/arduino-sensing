# -*- coding: utf-8 -*-
"""
Monitor voltages from pins A0 through A3 and A6, A7 alongside humidity and temperature from BME280

Isaac Zakaria
Gordon Guo
01 November 2021

Rev: 14 December 2022
Updated to monitor time from Arduino instead of from Python
"""

import serial
import time
import csv
import numpy as np

# port = '/dev/ttyACM0'
# port = 'COM4'   
port = '/dev/cu.usbmodem21201'

def figaroMonitor(filename, runtime, port=port, baudrate=9600, activePins=[0,1,2,3,4,5,6,7,8]):
    
    calibration = 0

    arduino = serial.Serial(port=port, timeout=1, baudrate=baudrate)
    
    with open(filename, 'w', newline='') as csvfile:
        fieldnames = ('t','0','1','2','3','6','7','H','T')
        readoutDict = {}
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        
        writer.writeheader()
        readoutDict['t'] = 0.0
        while True:
                b = arduino.readline()
                s = b.decode()
                # print(len(s))
                if len(s) == 0:
                    break
        while readoutDict['t'] <= runtime:

            for k in activePins:
                b = arduino.readline()
                s = b.decode()
                print(s)
                if len(s) != 0:
                    readoutDict[s[0]] = float(s[1:])
                # if calibration == 0:
                #     calibration = readoutDict['t']/1000;
                # readoutDict['t'] = readoutDict['t']/1000 - calibration
            writer.writerow(readoutDict)
            # readoutDict['t'] = time.monotonic() - t0
            # print('-')
    
    arduino.close()
    
figaroMonitor('test.csv', np.inf, port=port)