import time
import serial
import threading

ser=serial.Serial(
    port='/dev/ttyACM0',
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
    )

#Set global killswitch event across threads
stop_threads=threading.Event()

#lazy ... dont want to change these everywhere
input_string=">> "
error_msg="Invalid entry. Try again."
return_menu="Returning to main menu."

#For error handling
#ensure user input is int or str before using logic
def isInt(string):
    try: 
        int(string)
        return True
    except ValueError:
        return False

# Walk through Calibration process.
# mbed is only streaming data; no special pi<>mbed<>bno interaction.
# Cal bits per CALIB_STAT register: 
#   ff - pass final, 3x - pass gyro, x3 - pass mag, xc - pass acc
def set_cal():
    returnval=0
    check_full=0xff
    while (returnval!=1):
        if ((calibration & check_full) == 0xff):
            print("Calibration completed.")
            print()
            returnval=1
        elif ((calibration & 0x3f) != 0x3f):
            gyro_check=0x30
            acc_check=0x0c
            mag_check=0x03

            #Accelerometer calibration (0x0c)
            while ((acc_check & calibration)!=acc_check):
				print()
                print("Calibrating Accelerometer:")
                print("Place vehicle in 6 different, orthogonal, and stable positions for a few seconds at a time.")
                print("Desired: 0x0c, Cal: "+str(hex(calibration)) + ", Mag bits: "+str(hex(calibration&acc_check)))
                time.sleep(0.5)
            print("Accelerometer calibrated.")

            #Magnetometer calibration (0x03)
            while ((mag_check & calibration)!=mag_check):
                print()
				print("Calibrating Magnetometer:")
                print("Do a barrel roll for magnetometer cal.")
                print("Desired: 0x03, Cal: "+str(hex(calibration)) + " Mag bits: "+str(hex(calibration&mag_check)))
                time.sleep(0.5)
            print("Magnetometer calibrated.")

            #Gyroscope calibration (0x30)
            while ((gyro_check & calibration) != gyro_check):
                print()
				print("Calibrating Gyroscope:")
                print("Place vehicle in a stable position for a few seconds at a time.")
                print("Desired: 0x30, Cal: "+str(hex(calibration)) + ", Mag bits: "+str(hex(calibration&gyro_check)))
                time.sleep(0.5)
            print("Gyroscope calibrated.")          

        time.sleep(0.02)
    return returnval

def set_pos():
    returnval=0
    prefix='pos:'
    while(returnval!=1):
        print()
		print("Current position: " + str(status & 0x0007))
		print("Selection position: [0-7] or [r]eturn to main menu.")
        print("See datasheet for guidance.")
        selection=input(input_string)
        if (selection=='r'):
            returnval=1
        elif (isInt(selection)):
            if (int(selection)>=0) and (int(selection)<8):
                writeline=(prefix+"00"+selection).encode()
				print()
                print("sent: "+writeline.decode())
                ser.write(writeline)
				time.sleep(0.1)
				current_pos=(status & 0x0007)
                if (current_pos==int(selection)):
					print("Success. BNO reported position " + str(current_pos))
					returnval=1
				else: print("Mismatch. BNO reported position " + str(current_pos))
        else:
			print(error_msg)
    return returnval

def set_offsets():
    returnval=1
	print()
    print("Not setup yet.")
    return returnval

def config_menu():
    selection=0
    while(selection!=1):
        print()
        print("Status: "+str(status))
        print("Config select: [c]alibration, [p]osition, [o]ffsets, [r]eturn for main menu")
        configsel=input(input_string)
        if configsel=='c':
            selection=set_cal()
        elif configsel=='p':
            selection=set_pos()
        elif configsel=='o':
            selection=set_offsets()
        elif configsel=='r':
            selection=1
        else:
            selection=0            
            print (error_msg)
        time.sleep(0.005)
    return selection

def get_angles():
    #Set key values.
    #Keys are first 4 Hexadecimal values in line
    #and used to "tag" what kind of data follows
    ver_key=0x1234
    status_key=0xffff  #also 'ready'
    cal_key=0xc000
    h_key=0xc100
    r_key=0xc300
    p_key=0xc500

    #Set globals accessible outside this thread
    global status
    global calibration
    global heading
    global roll
    global pitch

    #Initialize values to 0
    status,calibration,heading,roll,pitch=0,0,0,0,0
    key,st,cal,h,r,p=0x0,0x0,0x0,0x0,0x0,0x0
    
    #See mbed code. Roll and Pitch are +/- 180 from BNO
    #Offset allows mbed and raspi to deal strictly with
    #positive numbers.
    #360 deg added on mbed side, must be subtracted
    offset=0x1680

    #mbed sends 8bit words, +1 for overhead
    word_size=9

    while not stop_threads.is_set(): 
        #Only proceed if there are bytes in Serial waiting to be read
        while (ser.inWaiting==0):
            time.sleep(0.005)
            print("no serial input... waiting")
            
        #Read bytes from Serial
        in_buffer=ser.readline()
        
        #Only assign prefix and data if serial line is correct length
        #Prevents Value Errors
        if (len(in_buffer)==word_size):
            str_buffer_prefix=in_buffer[0:2].decode()+in_buffer[2:4].decode()
            int_buffer_prefix=int(str_buffer_prefix,base=16)
            str_buffer_data=in_buffer[4:6].decode()+in_buffer[6:8].decode()
            int_buffer_data=int(str_buffer_data,base=16)
        else:
            int_buffer_prefix=0x0
            int_buffer_data=0x0

        #Sort data to correct variable using the keys
        if (int_buffer_prefix == ver_key):
            key=int_buffer_data
            #print("key detected")
        if (int_buffer_prefix == status_key):
            st=int_buffer_data
            # print("status detected")
        if (int_buffer_prefix == cal_key):
            cal=int_buffer_data
            #print("cal detected")
        if (int_buffer_prefix == h_key):
            h= int_buffer_data/16
            #print("heading detected")
        if (int_buffer_prefix == r_key):
            r=(int_buffer_data-offset)/(0x10)
            #print("roll detected")
        if (int_buffer_prefix == p_key):
            p=(int_buffer_data-offset)/(0x10)
            #print("pitch detected")

#Only update globals after key and status are verified
        if (key==0xabcd) and (int_buffer_prefix==status_key):
            #print("VERIFIED")
            heading = h
            roll = r
            pitch = p
            calibration=cal
            status=st
#reset verifications for next loop
            key=0x0
            #print ("cal:" + str(cal) + " heading:"+str(h)+" roll:"+str(r)+" pitch:"+str(p))
#mbed sends 6 8bit words every 20ms
#Or one word around every 3ms
#This while loop only processes 1 word at a time
        time.sleep(0.001)

def operate():
    print("heading: " +str(heading))
    print("calibration: " +str(hex(calibration)) + " status: "+str(hex(status)))
    compare=heading
    target=-1
    success=0

    #Acceptable tolerance in degrees
    #mbed has separate tolerance (likely set tighter)
    tolerance=1

    while (target==-1):
        print()
		print("Heading: " + str(heading) + " Cal: " + str(hex(calibration)))
        print("Enter heading [0 to 360] or [r]eturn to Main Menu")
        decision=input(input_string)
        if (decision=='r'):
            target=compare
            success=1
        elif (isInt(decision)):
            headingselect=int(decision)
            if (headingselect>=0) and (headingselect<=360):
                target=headingselect
                if (headingselect<10): prefix='hea:00'
                elif (headingselect<100): prefix='hea:0'
                else: prefix='hea:'
                writeline=(prefix+decision).encode()
                print("sent: "+writeline.decode())
                ser.write(writeline)
            else: print(error_msg)
        else: print(error_msg)
    while (success!=1):
        stopped=0
        compare=heading
        diff=abs(target-compare)
        if (diff>180):
            if (target>180): diff=((compare+180)-(target-180))
            if (compare>180): diff=((target+180)-(compare-180))
        time.sleep(0.02)
        print("heading: "+str(heading))
        if ((status & 0x18)==0):
            stopped = 1
            print("Stopped")
        elif ((status & 0x8)==0x8):
            print("Turning Right")
        elif ((status & 0x10)==0x10):
            print("Turning Left")
        print("diff: " + str(diff) + " heading: " +str(heading))
        print("cal: " +str(hex(calibration)) + " status: " +str(hex(status)))
		print()
        if ((diff<=tolerance) and (stopped==1)):
            print("Success.")
            success=1
    return success
    
#Main menu
#outsource selection to individual functions
def main():
    #Open separate thread for intaking Serial data stream from mbed
    ReadSerial_thread=threading.Thread(target=get_angles,args=())
    ReadSerial_thread.start()

    quit=0
    while (quit!=1):
        print()
        print("Mode select: [c]onfig, [m]onitor, [o]perate, [q]uit")
        modesel=input(input_string)
        if modesel=='c':
            result=config_menu()
            if (result):
                print(return_menu)
        elif modesel=='m':
            while 1:
                print("VERIFIED HEADING: " + str(heading) + " Cal: " +str(hex(calibration)))
                print("roll: "+str(roll)+" pitch: "+str(pitch))
                print("Status: " + str(hex(status)))
                time.sleep(0.02)
        elif modesel=='o':
            result=operate()
            if (result):
                print(return_menu)
        elif modesel=='q':
            quit=1
        else:
            print(error_msg)
    stop_threads.set()
    time.sleep(0.5)
    return 1

main()

