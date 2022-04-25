#run with sudo privileges
#add user to the root group:
#	sudo usermod -a -G root pi
#
#create permissions file using nano
#	sudo nano /etc/udev/rules.d/55-permissions-uinput.rules
#	enter rules:
#		

from datetime import datetime
import paho.mqtt.client as MQTT
import xbox
from math import sqrt, pi, atan
from time import sleep,asctime
from gc import collect as trash
import threading

js = xbox.Joystick()
client = MQTT.Client("Pats Desk")

file={}

def read_json(filename):
	global file
	data={}
	try:
		# with open(filename) as file:
		data = json.load(file)
	except:
		pass
	return data
	#print(values)

def ingest(file_name):
	process=0
	xbox_data={
		'EVENTHORIZON':0,
		'zeroize':0,
		'breach':0,
		'pitch':0,
		'roll':0,
		'hea_stop':0,
		'dep_stop':0,
		'vel_stop':0,
		'all_stop':0,
		'reset':0,
		'dep':4,
		'hea':4,
		'vel':4
	}
	try:
		read_data=read_json(file_name)
		for key in xbox_data:
			xbox_data[key]=read_data[key]
		process=1
	except:
		pass
	final=0x000000
	if (process):
		print(xbox_data)
		stop_data = (xbox_data['all_stop']*0x8) + (xbox_data['reset']*0x4) + (xbox_data['EVENTHORIZON']*0x2)
		if not (stop_data):
			final+=((xbox_data['zeroize']*0x4)+xbox_data['breach']<<16)
			final+=(((xbox_data['roll']<<2) + xbox_data['pitch'])<<12)
			# final+=(xbox_data['pitch']<<12)
			final+=((xbox_data['hea_stop']*0x8 + xbox_data['hea'])<<8)
			final+=((xbox_data['dep_stop']*0x8 + xbox_data['dep'])<<4)
			final+=((xbox_data['vel_stop']*0x8 + xbox_data['vel']))
		final+=(stop_data<<20)
	return final

def check_quit():
	return 1

def mainMQTT():
	q=0
	try:
		client.connect('127.0.0.1')
		q=1
	except:
		print("didn't connect")
	# xbox_file="/home/pi/xbox/xbox_read.txt"
	while(q):
		client.publish('timestamp',str(datetime.now()))
		client.publish('uuv/move',ingest(xbox_file))
		sleep(0.055)
		trash()
		q=check_quit()


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
	global file
	# filename="/home/pi/xbox/xbox_read.txt"
	# file=open(filename,'w')
	writedata=process()
	json.dump(writedata,file)
	# file.close()
	#print(writedata)
	trash()

def startMQTTthread():
	thread = threading.Threat(target=mainMQTT)
	thread.start()

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



