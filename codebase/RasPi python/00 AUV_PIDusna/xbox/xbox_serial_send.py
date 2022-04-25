from time import sleep
from gc import collect as trash
import serial
import json

#                       #
#------Serial Setup-----#
#                       #
ser=serial.Serial(
    port='/dev/ttyACM0',
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
    )
# writeline=('string').encode()
# ser.write(writeline)

xbox_data={
	'sto':None,
	'pit':None,
	'rol':None,
	'dep':None,
	'hea':None,
	'vel':None,
	'off':None,
	'zer':None,
}

last_xbox_data={
	'sto':None,
	'pit':None,
	'rol':None,
	'dep':None,
	'hea':None,
	'vel':None,
	'off':None,
	'zer':None,
}


def write_serial(writeline):
	try:
		ser.write(writeline.encode())
		print(writeline)
		sleep(0.03)
	except:
		pass
		# print("serial write failed: "+str(writeline))


def process():
	for key in xbox_data:
		if (xbox_data[key]):
			ser_line=key+":"+xbox_data[key]
			if not ((ser_line=="hea:834") or (ser_line=="dep:824") or (ser_line=="hea:830") or (ser_line=="dep:820")):
				write_serial(ser_line)

def trim_xbox_data()
	for key in xbox_data:
		if (xbox_data[key]==last_xbox_data[key]):
			xbox_data[key]=None
		else:
			last_xbox_data[key]=xbox_data[key]

def read_json():
	filename="/home/pi/xbox/serial_write.txt"
	data={}
	# file=open(filename,'r')
	# data = json.load(file)
	try:
		with open(filename) as file:
			data = json.load(file)
	except:
		pass
		# print("fail to read file")
	# print(data)
	return data

def ingest():
	success=0
	read_data=read_json()
	try:
		for key in xbox_data:
			xbox_data[key]=read_data[key]
		success=1
	except:
		pass
		# print("fail to load json from file")
	if (success):
		trim_xbox_data()
		process()

def check_quit():
	return 1

def main():
	q=0
	q=check_quit()
	while(q):
		q=check_quit()
		ingest()
		sleep(0.05)

main()















