import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(17, GPIO.OUT)
GPIO.output(17,1)

def hardreset():
	time.sleep(0.1)
	GPIO.output(17,0)
	time.sleep(1)
	GPIO.output(17,1)

def main():
	hardreset()

main()
