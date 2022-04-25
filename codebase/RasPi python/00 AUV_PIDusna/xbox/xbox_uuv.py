#run with sudo privileges
#add user to the root group:
#	sudo usermod -a -G root pi
#
#create permissions file using nano
#	sudo nano /etc/udev/rules.d/55-permissions-uinput.rules
#	enter rules:
#		

import xbox
from math import sqrt, pi, atan
from time import sleep
import json
from gc import collect as trash
js = xbox.Joystick()

def scalar(a,b):
	# rescale_factor = 1.414213562373095
	returnval = sqrt(a**2+b**2)
	return returnval
def angle(a,b):
	degree_conversion = 360 / (2*pi)
	if (a==0):
		a=0.000001
	returnval = atan(b/a)
	if (a < 0):
		returnval += pi
	elif (b < 0):
		returnval += 2*pi
	return returnval * degree_conversion
# def relative(x):
	# returnval = (round(x*3))+834
	# return returnval

def process():
	jvals=js.sample()
	values={}
	values={
		'EVENTHORIZON':(jvals['leftBumper'] and jvals['rightBumper'] and jvals['A'] and jvals['B'] and jvals['Y'] and jvals['X']),
		'zeroize':(jvals['X'] and jvals['Y']),
		'breach':jvals['dpadUp'],
		'pitch':(jvals['leftBumper']*2)+jvals['rightBumper'],
		'roll':(jvals['leftTrigger']*2)+jvals['rightTrigger'],
		'hea_stop':jvals['leftThumbstick'],
		'dep_stop':jvals['rightThumbstick'],
		'vel_stop': int(not (round(jvals['left_y'])*3)),
		'all_stop':(jvals['leftThumbstick'] and jvals['rightThumbstick']),
		'reset':(jvals['Start'] and jvals['Back']),
		# 'scalar1':scalar(jvals['x1'],jvals['y1']),
		# 'vector1':angle(jvals['y1'],jvals['x1']),
		# 'scalar2':scalar(jvals['x2'],jvals['y2']),
		# 'vector2':angle(jvals['y2'],jvals['x2']),
		# 'vector2_x2':relative(x2),
		'dep':round(jvals['right_y']*-3)+4,
		'hea':round(jvals['right_x']*3)+4,
		'vel':round(jvals['left_y']*3)+4
	}
	#print(jvals)
	#print(values)
	return values

def writeToFile():
	filename="/home/pi/xbox/xbox_read.txt"
	file=open(filename,'w')
	writedata=process()
	json.dump(writedata,file)
	file.close()
	#print(writedata)
	trash()

def closeXbox():
	sleep(0.1)
	js.close()

def main():
	fail=0
	while(fail<100):
		try:
			js.sample()
			fail=0
		except:
			fail+=1
			print("try failed: " + str(fail))
		if (fail==0):
			writeToFile()
			sleep(0.05)
		sleep(0.1)
	closeXbox()


main()


