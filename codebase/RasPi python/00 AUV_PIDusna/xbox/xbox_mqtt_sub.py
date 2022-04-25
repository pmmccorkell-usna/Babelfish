from time import sleep
import RPi.GPIO as GPIO
import paho.mqtt.client as MQTT
import logging
import logging.handlers
from datetime import datetime
from gc import collect as trash
import json
import os
from server import *

DEBUGGING=0
speed=int(270/3)

#                       #
#-----Logging Setup-----#
#                       #
filename=datetime.now().strftime('/home/pi/xbox/mqtt_%Y%m%d_%H:%M:%s.log')
log = logging.getLogger()
log.setLevel(logging.INFO)
format = logging.Formatter('%(asctime)s : %(message)s')
file_handler = logging.FileHandler(filename)
file_handler.setLevel(logging.INFO)
file_handler.setFormatter(format)
log.addHandler(file_handler)

#                       #
#-------MQTT Setup------#
#                       #
client=MQTT.Client("red dwarf")
subtopic='uuv/move'
timetopic='timestamp'
timestamp=0

#
# IPs for different MQTT server Pis.
# This is a backup to the import server line, which stores a static server.
# Uncommenting a line will override the server.py setting.
# Only uncomment one, depending on which server you desire.
#
# server="127.0.0.1"	# loopback
# server="172.30.35.102"	# Pat's desk
# server="172.30.38.181"	# SURF

def hardreset():
	sleep(0.1)
	GPIO.output(17,0)
	sleep(1)
	GPIO.output(17,1)

def reset_ser():
	data={
		'sto':None,
		'pit':None,
		'rol':None,
		'dep':None,
		'hea':None,
		'vel':None,
		'off':None,
		'zer':None,
	}
	return data

#a basic callback for MQTT that prints message data directly.
def on_message(client,userdata,message):
	print()
	print("mqtt rx:")
	print(message.topic)
	print(message.qos)
	print(message.payload)
	print(message.payload.decode())
	print(message.retain)
	print(client)

#a basic callback for MQTT that stores message data to a log file.
def log_message(client,userdata,message):
	log.info("message rx")
	log.info(str(message.topic)+", "+str(message.payload))

def write_serial(dictionary):
	filename="/home/pi/xbox/serial_write.txt"
	file=open(filename,'w')
	json.dump(dictionary,file)
	file.close()
	#print(dictionary)

def move1(word,mode):
	offset={
		'rol':800,
		'pit':810,
		'dep':820,
		'hea':830,
		'vel':840,
		'off':850
	}
	returnval = None
	# if (DEBUGGING):
		# print("word: "+str(word)+", mode: "+str(mode))
	if (word&0x8):
		returnval='999'
	# elif (word==0x4):
		# returnval = None
	elif (word) and (word!=0x4):
		# print(word)
		returnval=str(offset[mode]+(word&0x7))
	return returnval

def move2(word):
	switch={
		0:None,
		1:'817',
		2:'811',
		3:'999',
		4:'807',
		8:'801',
		12:'999'
	}
	returnval=[]
	returnval.append(switch[(word&0x3)])
	returnval.append(switch[(word&0xC)])
	return returnval

def move3(word):
	if (word&0x8):
		returnval='999'
	elif (word):
		returnval=str(400+(((word&0x7)-4)*speed))
	return returnval

def special(word):
	global DEBUGGING
	returnval='000'
	if (word&0x4):
		log.info("mbed reset initiated.")
		print("RESET MBED")
		if not (DEBUGGING):
			hardreset()
		log.info("mbed reset completed.")
	if (word&0x2):
		print("EVENT HORIZON")
		log.info("EVENT HORIZON. See you on the flipside.")
		if not (DEBUGGING):
			os.system("sudo shutdown -h now")
		log.info("If you see this, Event Horizon was net executed.")
	return returnval

#the callback that our program will use to control device over serial.
def process(client,userdata,message):
	global timestamp,DEBUGGING
	ser_data=reset_ser()
	msg=message.payload.decode()
	topic=message.topic
	data=int(msg)
	if (data & 0xE00000):
		ser_data['sto']=special(data>>20)
	else:
		ser_data['hea']=move1((data&0xF00)>>8,'hea')
		if (data&0x40000):
			ser_data['zer']='000'
		if (data&0x10000):
			log.info("breach")
			ser_data['dep']='020'
		else:
			ser_data['dep']=move1((data&0xF0)>>4,'dep')
		ser_data['vel']=move3(data&0xF)
	temp=move2((data&0xF000)>>12)
	ser_data['pit']=temp[0]
	ser_data['rol']=temp[1]
	if (DEBUGGING):
		print(msg)
		if (topic==timetopic):
			timestamp=msg
		print(ser_data)
		log.info("tx_timestamp: "+str(timestamp)+" rx_timestamp: "+str(datetime.now()))
	write_serial(ser_data)

def check_quit():
	return 1

def setup_subscription():
	q=0
	try:
		client.connect(server)
		# client.on_message=on_message		#for debugging
		client.on_message=process
		client.subscribe(subtopic)
		client.subscribe(timetopic)		#for end-to-end timing
		client.loop_start()
		log.info("mqtt subscription script started")
		q=1
	except:
		print("didn't connect")
	return q

def main():
	q=0
	q=setup_subscription()
	
	#GPIO things for resetting the mbed.
	GPIO.setmode(GPIO.BCM)
	GPIO.setwarnings(False)
	GPIO.setup(17, GPIO.OUT)
	GPIO.output(17,1)
	while(q):
		trash()
		sleep(1)
		q=check_quit()

main()




