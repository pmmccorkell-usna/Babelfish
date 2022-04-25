#run with sudo privileges
#add user to the root group:
#	sudo usermod -a -G root pi
#
#create permissions file using nano
#	sudo nano /etc/udev/rules.d/55-permissions-uinput.rules
#	enter rules:
#		
import json
from time import sleep,asctime
from datetime import datetime
import paho.mqtt.client as MQTT
from gc import collect as trash

client = MQTT.Client("Pats Desk")

def read_json(filename):
	data={}
	try:
		with open(filename) as file:
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

def main():
	q=0
	try:
		client.connect('127.0.0.1')
		q=1
	except:
		print("didn't connect")
	xbox_file="/home/pi/xbox/xbox_read.txt"
	while(q):
		client.publish('timestamp',str(datetime.now()))
		client.publish('uuv/move',ingest(xbox_file))
		sleep(0.055)
		trash()
		q=check_quit()

main()


