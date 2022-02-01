# -*- coding: utf-8 -*-
"""
Isaac Zakaria
1 February 2022

Rev: 1 February 2022
Updated to monitor time from Arduino instead of from Python.
Fixed issues where data logging isn't synchronized with Arduino output, causing premature termination.
Added comments.
"""

import serial
import csv
import numpy as np

# port = '/dev/ttyACM0' # Debian
# port = 'COM4' # Windows
port = '/dev/cu.usbmodem21201' # MacOS

def figaroMonitor(filename, runtime, port=port, baudrate=115200, figaro=True, bme=680, scd=False):
    """
    filename: CSV file to save to
    runtime: desired runtime in minutes
    port: serial port to listen to
    figaro: if True, get voltage readout from analog pins
    """
    
    runtime = runtime*60*10**3 # convert runtime from minutes to milliseconds
    
    t0 = np.inf
    t = 0

    arduino = serial.Serial(port=port, timeout=1, baudrate=baudrate) # initialize arduino object
    
    with open(filename, 'w', newline='') as csvfile: # generate CSV to store data
        fieldnames = ["t "]
        if figaro:
            fieldnames = fieldnames + ["0 ","1 ","2 ","3 ","6 ","7 "]
        if bme == 280:
            fieldnames = fieldnames + ["HB","TB"]
        elif bme == 680 or bme == 688:
            fieldnames = fieldnames + ["HB","TB","RB"]
        if scd:
            fieldnames = fieldnames + ["HS","TS","CS"]
        readoutDict = {}
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        
        writer.writeheader()
        
        print("CSV initialied with headers")
        print(fieldnames)
        while True:
                b = arduino.readline() # get serial output
                s = b.decode() # generate string from serial output
                if s[0] == "x": # don't start logging until the beginning of a complete chunk of data
                    break
        while t - t0 <= runtime: # note: Arduino reports absolute time in milliseconds
            for k in range(len(fieldnames)):
                b = arduino.readline()
                s = b.decode()
                print(s)
                if len(s) != 0 and s[0] != "x": # prevents Python from trying to record empty lines in serial output
                    readoutDict[s[0:2]] = float(s[2:])
                t = readoutDict["t "]
                if t0 == np.inf:
                    t0 = np.copy(t) # save first absolute time for tracking runtime
             
            writer.writerow(readoutDict)
    
    arduino.close()
    
figaroMonitor('null.csv', 0.25, port=port, figaro=False, bme=280, scd=False)