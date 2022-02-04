# -*- coding: utf-8 -*-
"""
Isaac Zakaria
4 February 2022

Rev: 4 February 2022
"""

import serial
import csv
import numpy as np

# port = '/dev/ttyACM0' # Debian
# port = 'COM4' # Windows
port = '/dev/cu.usbmodem21401' # MacOS

def figaroMonitor(filename, runtime, port=port, baudrate=115200, analogPins=False, bme=280, scd=False, verbose=False):
    """
    filename: CSV file to save to
    runtime: desired runtime in minutes
    port: serial port to listen to
    analogPins: if True, get voltage readout from analog pins
    bme: if 280, get RH and T from Bosch BME280; if 680, get RH, T, and R from Bosch BME680/688
    scd: if True, get RH, T, and CO2 PPM from Sensirion SCD30
    verbose: if True, print sensor readouts to console
    """
    
    runtime = runtime*60*10**3 # convert runtime from minutes to milliseconds
    
    print("sensing run initialized with runtime")
    print(runtime)
    
    t0 = np.inf # initialize initial time
    dt = 0 # runtime counter

    arduino = serial.Serial(port=port, timeout=1, baudrate=baudrate) # initialize arduino object
    
    with open(filename, 'w', newline='') as csvfile: # generate CSV to store data
        fieldnames = ["t "]
        if analogPins: # monitor analog pins
            fieldnames = fieldnames + ["0 ","1 ","2 ","3 ","6 ","7 "]
        if bme == 280:
            fieldnames = fieldnames + ["HB","TB"]
        elif bme == 680 or bme == 688:
            fieldnames = fieldnames + ["HB","TB","RB"]
        if scd:
            fieldnames = fieldnames + ["HS","TS","CS"]
        
        # create CSV
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()
        print("CSV initialied with headers")
        print(fieldnames)
        
        # initialize dictionary
        readoutDict = dict.fromkeys(fieldnames, None)
        while True:
                b = arduino.readline() # get serial output
                s = b.decode() # generate string from serial output
                if len(s) != 0 and s[0] == "x": # don't start logging until the beginning of a complete chunk of data
                    break
        while dt <= runtime: # note: Arduino reports absolute time in milliseconds
            while True:
                b = arduino.readline()
                s = b.decode()
                if len(s) != 0 and s[0] == "y": # stop logging when chunk end is encountered
                    break
                elif len(s) !=0 and s[:2] in fieldnames: # only log data from desired pins and/or sensors
                    readoutDict[s[:2]] = float(s[2:])
                    print(s)
            if t0 == np.inf: # save initial time for runtime monitoring
                t0 = readoutDict["t "]
            writer.writerow(readoutDict)
            dt = readoutDict["t "] - t0 # compute current runtime
            print("runtime (min):")
            print(dt*(10**-3)/60)
            print("")

    arduino.close()
    
figaroMonitor('null.csv', 0.1, port=port, analogPins=True, bme=680, scd=True)