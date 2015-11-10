#!/usr/bin/env python
# /* -*-  indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */
# /*
# Copyright (c) 2013, Daniel M. Lofaro
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the author nor the names of its contributors may
#       be used to endorse or promote products derived from this software
#       without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Based on: https://github.com/thedancomplex/pydynamixel
# */

import sys
#sys.path.append('/home/student/projects_wrk/pydynamixel-master/dynamixel')
import os
#import dynamixel
#import serial_stream
import math
import time
import random
import sys
import subprocess
import optparse
#import yaml
#import dynamixel_network
import numpy as np
import socket
import serial
import time
import ach
import critr_ach
import csv
comm_channel = ach.Channel(critr_ach.CRITR_CHAN_REF_NAME)
comm_channel.flush()
reference_struct = critr_ach.CRITR_REF()

#############################################################################################
##	Radian to Dynamixel conversion functions
def rad2dyn(rad):
    return np.int(np.floor( (rad + np.pi)/(2.0 * np.pi) * 1023 ))

def dyn2rad(en):
    return ((en*2.0*np.pi)/1024) - np.pi

##############################################################################################


Base_36_Table=['0','1','2','3','4','5','6','7','8','9','a','b','c','d','e','f','g','h','i','j','k','l','m','n','o','p','q','r','s','t','u','v','w','x','y','z']
data=[]



def main():
    #portName = '/dev/ttyACM1'
    #baudRate = 1000000 #settings['baudRate']			
    #highestServoId = settings['highestServoId']
    #seriall = serial.Serial(port=portName, baudrate=baudRate, timeout=1)
    # Ping the range of servos that are attached
    time.sleep(1)
    start_time=0
    receive_time=0
    counter = 0
    while True:
	start_time = time.time()
        #data, addr = s.recvfrom(1024)			#gets positions from the client
        #data = data.split()				# splits the string at spaces
	string_send = ''
	num = 0 
	datastring = ''
	[status, framesize] = comm_channel.get(reference_struct, wait=True, last=True)
	#time.sleep(.01)
	latency = time.time() - start_time - 1/60
	#print time.time()
	data.append([latency])
	if counter == 1000:
		with open('latency.csv', 'w') as fp:
    			a = csv.writer(fp, delimiter=',')
    			a.writerows(data)
			#a.close()
		print "FILE PRINTED"
	counter = counter + 1
	#for data in reference_struct.ref:
	#	string_send = string_send + str(data) + ' '
	#	first_byte = math.floor(float(data)/36)
        #        second_byte = data%36
        #        datastring = datastring + Base_36_Table[int(first_byte)] + Base_36_Table[int(second_byte)] 
	#print string_send
	#seriall.write(datastring)    	
	#time.sleep(.0001)
        #msg = seriall.read(seriall.inWaiting())
	#print msg
    c.close()
main()
def validateInput(userInput, rangeMin, rangeMax):
    '''
    Returns valid user input or None
    '''
    try:
        inTest = int(userInput)
        if inTest < rangeMin or inTest > rangeMax:
            print "ERROR: Value out of range [" + str(rangeMin) + '-' + str(rangeMax) + "]"
            return None
    except ValueError:
        print("ERROR: Please enter an integer")
        return None
    
    return inTest
'''
if __name__ == '__main__':
    
    parser = optparse.OptionParser()
    parser.add_option("-c", "--clean",
                      action="store_true", dest="clean", default=False,
                      help="Ignore the settings.yaml file if it exists and \
                      prompt for new settings.")
    
    (options, args) = parser.parse_args()
    
    # Look for a settings.yaml file
    settingsFile = 'settings.yaml'
    if not options.clean and os.path.exists(settingsFile):
        with open(settingsFile, 'r') as fh:
            settings = yaml.load(fh)
    # If we were asked to bypass, or don't have settings
    else:
        settings = {}
        if os.name == "posix":
            portPrompt = "Which port corresponds to your USB2Dynamixel? \n"
            # Get a list of ports that mention USB
            try:
                possiblePorts = subprocess.check_output('ls /dev/ | grep -i usb',
                                                        shell=True).split()
                possiblePorts = ['/dev/' + port for port in possiblePorts]
            except subprocess.CalledProcessError:
                sys.exit("USB2Dynamixel not found. Please connect one.")
                
            counter = 1
            portCount = len(possiblePorts)
            for port in possiblePorts:
                portPrompt += "\t" + str(counter) + " - " + port + "\n"
                counter += 1
            portPrompt += "Enter Choice: "
            portChoice = None
            while not portChoice:                
                portTest = raw_input(portPrompt)
                portTest = validateInput(portTest, 1, portCount)
                if portTest:
                    portChoice = possiblePorts[portTest - 1]

        else:
            portPrompt = "Please enter the port name to which the USB2Dynamixel is connected: "
            portChoice = raw_input(portPrompt)
    
        settings['port'] = portChoice
        
        # Baud rate
        baudRate = None
        while not baudRate:
            brTest = raw_input("Enter baud rate [Default: 1000000 bps]:")
            if not brTest:
                baudRate = 1000000
            else:
                baudRate = validateInput(brTest, 9600, 1000000)
                    
        settings['baudRate'] = baudRate
        
        # Servo ID
        highestServoId = None
        while not highestServoId:
            hsiTest = raw_input("Please enter the highest ID of the connected servos: ")
            highestServoId = validateInput(hsiTest, 1, 255)
        
        settings['highestServoId'] = highestServoId

    
    main(settings)
'''
