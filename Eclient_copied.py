import sys
sys.path.append('/home/student/ECE-499-590-Fall-2014/dynamixel-1.0.1/dynamixel')
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
#from ovrsdk import *
import socket
import thread
import math
import serial
#import syslog
import time
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
#host = '127.0.0.1'
host = '192.168.1.213' #'10.159.168.77'    #this is the computers current IP
port = 21111          #this is just a random port number
#server = ('127.0.0.1',20000)    #tells the computer where to listen and send
server = (host,20000)    #tells the computer where to listen and send
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)    #tells to use network/udp
while (x == 1):            #this makes it so that if a port is taken it will try for a new port
    try:
        print port
        s.bind((host, port))    #trys this port if it is taken then moves on to a new port
        x=0
    except IOError:
        port+=1
    else:
        port+=1


def main(settings):
	#portName = settings['port']        #searches for a usb2dyn
	portName = '/dev/ttyACM0' #'COM22'        #searches for a usb2dyn
	baudRate = 1000000             #this is set to max can be changed to =>settings['baudRate']<= to be asked for a speed
	highestServoId = settings['highestServoId']#asked for highest servo id
	ard = serial.Serial(port=portName, baudrate=baudRate, timeout=1)
	#net = dynamixel_network.DynamixelNetwork(seriall) # Ping the range of servos that are attached
	#print "Scanning for Dynamixels..."
	#net.scan(1, highestServoId)        #scans for all attached servos

	#myActuators = []            #starts a list
	#for dyn in net.get_dynamixels():    #makes a list of all dyn's
	#    print dyn.id
	#    myActuators.append(net[dyn.id])

	#if not myActuators:            #if the list has nothing show error
	#  print 'No Dynamixels Found!'
	#  sys.exit(0)
	#else:                #if it found some it prints done
	#  print "...Done"
	#for actuator in myActuators:    #this is just an initializing loop to tell the servos to move as fast as we want
	#    actuator.moving_speed = 1023
	#    actuator.synchronized = True
	#    actuator.torque_enable = True
	#actuator._set_torque_limit(0)

	#ovr_Initialize()            #this part is form the oculus rift(OR) sdk this gets the pan and tilt
	#hmd = ovrHmd_Create(0)
	#hmdDesc = ovrHmdDesc()
	#ovrHmd_GetDesc(hmd, byref(hmdDesc))
	#ovrHmd_StartSensor(hmd,ovrSensorCap_Orientation | ovrSensorCap_YawCorrection,0)#end this part of OR code
	#time.sleep(2) # wait for Arduino
	time.sleep(1)
	my_bytes = bytearray()
	my_bytes.append(highestServoId)
	ard.write(my_bytes)
	time.sleep(.5)
	#print highestServoId
	print 'start'
	while True:
		y = 1
		ard.flush()
		while(y==1):
			ard.write(my_bytes)
			#time.sleep(.1)
			time.sleep(.02+highestServoId * 0.005) # I shortened this to match the new value in your Arduino code
			# Serial read section
			msg = ard.read(ard.inWaiting()) # read all characters in buffer			
			#print msg			
			new = np.fromstring(msg, dtype=int, sep=' ')
			if (new.size == highestServoId+1):
				#print new
				if(sum(new[0:highestServoId]) == new[highestServoId]):
					y=0	
		s.sendto(msg, server)                            #sends the string made above
		print msg
		#time.sleep	(.1)
    	return None

def validateInput(userInput, rangeMin, rangeMax):                #setting up from here down
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
