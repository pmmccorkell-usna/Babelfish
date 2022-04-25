### Give web server user, 'www-data', access to GPIO ###
# sudo usermod -a -G gpio www-data
# sudo groups www-data
# sudo service apache2 restart


import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)
GPIO.output(17,1)

def main():
	time.sleep(0.1)
	GPIO.output(17,0)
	time.sleep(1)
	GPIO.output(17,1)

main()
