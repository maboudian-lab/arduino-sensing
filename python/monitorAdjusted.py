# -*- coding: utf-8 -*-
"""
Monitor voltages from pins A0 through A3 and A6, A7 alongside humidity and temperature from BME280

Isaac Zakaria
Gordon Guo
01 November 2021

Rev: 24 January 2022
Updated to monitor time from Arduino instead of from Python.
Fixed issues where data logging isn't synchronized with Arduino output, causing premature termination.
Added comments.
"""

import serial
import time
import csv
import numpy as np

# port = '/dev/ttyACM0' # Debian
port = 'COM3' # Windows
# port = '/dev/cu.usbmodem21201' # MacOS

def figaroMonitor(filename, runtime, port=port, baudrate=9600, activePins=[0,1,2,3,4,5,6,7,8]):
    """
    filename: CSV file to save to
    runtime: desired runtime in minutes
    port: serial port to listen to
    """
    
    runtime = (runtime/60)*10**(3) # convert runtime from minutes to milliseconds
    
    calibration = np.inf

    arduino = serial.Serial(port=port, timeout=1, baudrate=baudrate) # initialize arduino object
    
    with open(filename, 'w', newline='') as csvfile: # generate CSV to store data
        # fieldnames = ('t','0','1','2','3','6','7','H','T')
        fieldnames = ('t','0','1','2','3','6','7','HB','TB','RB','HS','TS','CS')
        readoutDict = {}
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        
        writer.writeheader()
        
        t = 0
        while True:
                b = arduino.readline() # get serial output
                s = b.decode() # generate string from serial output
                if len(s) == 0: # don't start logging until the beginning of a complete chunk of data
                    break
        while t - calibration <= runtime:
            for k in range(len(fieldnames)):
                b = arduino.readline()
                s = b.decode()
                print(s)
                if len(s) != 0: # prevents Python from trying to record empty lines in serial output
                    readoutDict[s[0]] = float(s[2:])
                if calibration != np.inf: # save first absolute time for tracking runtime
                    t = readoutDict['t']
                    calibration = t/1000
            writer.writerow(readoutDict)
    
    arduino.close()
    
figaroMonitor('null.csv', 1, port=port)