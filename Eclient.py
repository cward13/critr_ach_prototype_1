import sys
#sys.path.append('/home/student/ECE-499-590-Fall-2014/dynamixel-1.0.1/dynamixel')
import os
#import dynamixel
#import serial_stream
import time
import random
import sys
import subprocess
import optparse
#import yaml
#import dynamixel_network
import numpy as np
from ovrsdk import *
import socket
import thread
import math
import serial
#import syslog
import time
import ach
import critr_ach

import curses
import getopt

comm_channel = ach.Channel(critr_ach.CRITR_CHAN_REF_NAME)
comm_channel.flush()
reference_struct = critr_ach.CRITR_REF()



#############################################################################################
##    Radian to Dynamixel conversion functions
def rad2dyn(rad):
    return np.int(np.floor( (rad + np.pi)/(2.0 * np.pi) * 1023 ))

def dyn2rad(en):
    return ((en*2.0*np.pi)/1024) - np.pi

##############################################################################################
#global
#open a link to the server
x=1

def main(settings):
	portName = settings['port']        #searches for a usb2dyn
	#portName = 'COM22'        #searches for a usb2dyn
	baudRate = 1000000             #this is set to max can be changed to =>settings['baudRate']<= to be asked for a speed
	user_input = 0
	highestServoId = settings['highestServoId']#asked for highest servo id
	ard = serial.Serial(port=portName, baudrate=baudRate, timeout=1)
	time.sleep(1)
	my_bytes = bytearray()
	my_bytes.append(highestServoId)
	ard.write(my_bytes)
	time.sleep(.5)
	

	
        ovr_Initialize()            #this part is form the oculus rift(OR) sdk this gets the pan and tilt
        hmd = ovrHmd_Create(0)
        hmdDesc = ovrHmdDesc()
        ovrHmd_GetDesc(hmd, byref(hmdDesc))
        ovrHmd_StartSensor(hmd,ovrSensorCap_Orientation | ovrSensorCap_YawCorrection,0)#end this part of OR code


	print 'start'
	screen = curses.initscr()
	curses.noecho()
	curses.cbreak()
	screen.keypad(True)
	screen.timeout(10)
	user_input = 0
	print "Here"
	while True:
		y = 1
		ard.flush()
		while(y==1):
			char = screen.getch()
			if char == ord('q'):
				break
			elif char == curses.KEY_RIGHT:
				user_input = 4
			elif char == curses.KEY_LEFT:
				user_input = 3
			elif char == curses.KEY_UP:
				user_input = 1
			elif char == curses.KEY_DOWN:
				user_input = 2
			elif char == ord('s') or char == ord(' '):
				user_input = 0
			ard.write(my_bytes)
			time.sleep(.02+highestServoId * 0.005)
			msg = ard.read(ard.inWaiting())
			#print msg			
			new = np.fromstring(msg, dtype=int, sep=' ')
			#print msg
			if (new.size == highestServoId+1):
				#print new
				if(sum(new[0:highestServoId]) == new[highestServoId]):
					y=0	
		print "Here"
		#s.sendto(msg, server)                            #sends the string made above
		#load data into struct
		data = msg.split()
		for num in range(0,18):
			reference_struct.ref[num]=int(data[num])
		ss = ovrHmd_GetSensorState(hmd, ovr_GetTimeInSeconds())#more OR code
                pose = ss.Predicted.Pose
                reference_struct.ref[19] = rad2dyn(pose.Orientation.x*math.pi);                #gets tilt
                reference_struct.ref[18] = rad2dyn(pose.Orientation.y*math.pi);                #gets pan
		print msg
		reference_struct.dcref[0]=user_input
		comm_channel.put(reference_struct)
		#print reference_struct.dcref[0]
	return 1#None

def validateInput(userInput, rangeMin, rangeMax):
    try:
	inTest = int(userInput)
	if inTest < rangeMin or inTest > rangeMax:
		print "ERROR: Value out of range [" + str(rangeMin) + '-' + str(rangeMax) + "]"
		return None
    except ValueError:
	print("ERROR: Please enter an integer")
	return None
    return inTest

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
                possiblePorts = subprocess.check_output('ls /dev/ | grep -i ACM',
                                                        shell=True).split()
                possiblePorts = ['/dev/' + port for port in possiblePorts]
            except subprocess.CalledProcessError:
                sys.exit("Arm-Controller not found")
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
