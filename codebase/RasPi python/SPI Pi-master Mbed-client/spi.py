from time import sleep
from gc import collect as trash
import spidev

spiBus=0	# Bus0: mosi 19, miso 21, sclk 23
spiCS=0		# CS0 p24, CS1 p26
spi=spidev.SpiDev()
spi.open(spiBus,spiCS)
spi.max_speed_hz = 1000000	# 1 MHz (lpc1768 default)
spi.mode = 0b00		

def send_prefix():
	returnval=0
	prefix=0xc1
	check=spi.xfer2([prefix])
	print("check:"+str(check))
	if ((prefix-check[0])==1):
		returnval=1
	return returnval

# basic callback for MQTT that prints message data directly.
def process():
	#prefix=0xc100
	heading=0
	i=0
	while(i<360):
		writeline=[i]
		#writeline=[i*16]
		r=999
		if (send_prefix()):
			spi.xfer2(writeline)
			print("sent: "+str(writeline))
		print()
		#print("reply: "+str(r))
		print()
		i+=1
		sleep(0.1)

def main():
	while(1):
		process()
		trash()

main()
