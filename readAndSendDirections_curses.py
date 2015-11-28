import curses
import RPi.GPIO as GPIO
import sys, getopt
import serial
import time


# required setup
port = serial.Serial("/dev/ttyAMA0", baudrate = 9600, timeout = 0)
port.open();


screen = curses.initscr()
curses.noecho()
curses.cbreak()
screen.keypad(True)


# main/processing 
try:
	while True:
		char = screen.getch()
		screen.addstr(0,0,'     ')
		screen.addstr(0,1,'                   ')
		if char == ord('q'):
			break
		elif char == curses.KEY_RIGHT:
			screen.addstr(0,0,'right')
			user_input = "R"
		elif char == curses.KEY_LEFT:
			screen.addstr(0,0,'left')
			user_input = "L"
		elif char == curses.KEY_UP:
			screen.addstr(0,0,'up')
			user_input = "U"
		elif char == curses.KEY_DOWN:
			screen.addstr(0,0,'down')
			user_input = "D"
		elif char == ord('s') or char == ord(' '):
			screen.addstr(0,0,'stop')
			user_input = "S"
#		elif char == curses.KEY_DOWN and char == curses.KEY_LEFT:
#			screen.addstr(0,0,'down & left')
#			user_input = "DL"
#		elif char == curses.KEY_DOWN and char == curses.KEY_RIGHT:
#			screen.addstr(0,0,'down & right')
#			user_input = "DR"
#		elif char == curses.KEY_UP and char == curses.KEY_LEFT:
#			screen.addstr(0,0,'up & left')
#			user_input = "UL"
#		elif char == curses.KEY_UP and char == curses.KEY_RIGHT:
#			screen.addstr(0,0,'up & right')
#			user_input = "UR"
		port.write(user_input)
		port.flush()
		screen.addstr(1,0,port.read(port.inWaiting()))

# required breakdown
finally:
	curses.nocbreak();
	screen.keypad(0);
	curses.echo()
	curses.endwin()
	port.close()

