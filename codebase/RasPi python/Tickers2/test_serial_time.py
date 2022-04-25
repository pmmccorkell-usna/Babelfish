import time
import serial
import threading
import logging
import logging.handlers
from datetime import datetime
from tkinter import *
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)
GPIO.setup(40, GPIO.OUT)
GPIO.output(40,0)

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

#                       #
#-----Logging Setup-----#
#                       #
#filename = datetime.now().strftime('./log/AUV_%Y%m%d_%H:%M:%s.log')
filename=datetime.now().strftime('/var/www/auv_logs/AUV_%Y%m%d_%H:%M:%s.log')
log = logging.getLogger()
log.setLevel(logging.INFO)
format = logging.Formatter('%(asctime)s : %(message)s')
file_handler = logging.FileHandler(filename)
file_handler.setLevel(logging.INFO)
file_handler.setFormatter(format)
log.addHandler(file_handler)


#Set global killswitch event across threads
stop_threads=threading.Event()
quit_window=threading.Event()
stop_persistent_h=threading.Event()
stop_persistent_d=threading.Event()
suppress_commands=threading.Event()
mbed_in_commands=threading.Event()
mbed_in_call=threading.Event()

#lazy ... dont want to change these everywhere
input_string=">> "
error_msg="Invalid entry. Try again."
return_menu="Returning to main menu."
stop_command="STOP"

def horizon():
    writeline=stop_command.encode()
    ser.write(writeline)
    log.info("Raspi command sent: "+writeline.decode())
    print("sent: "+writeline.decode())
    print("Event Horizon initiated. Waiting for confirmation from mbed.")
    success=0
    i=0
    while(success==0):
        while(horizon_state==1):
            if (i==10):
                print("mbed confirmed entering Event Horizon.")
                print("System will remain locked out until mbed program releases.")
                print("Please stand by to stand by.")
            time.sleep(0.001)
            i+=1
        if ((i>0) and (horizon_state==0)):
            print()
            print("mbed has returned to normal state.")
            print("mbed was in Event Horizon state for " + str(i) + " ms.")
            success=1
    print("This action has been called " + str(horizon_count) + " times.")
    if (horizon_count>4):
        print("Raspi can only count total 15 events.")
        print("Recommend restarting mbed after 5 events.")
        print()
        print("Do you wish to reset mbed?")
        print("[y]es or [n]o")
        success=0
        reset=0
        selection=input(input_string)
        while (success==0):
            if (selection==stop_command):
                horizon()
                success=1
            elif (selection=='y'):
                success=reset_mbed()

def clear_serial():
    ser.reset_input_buffer()
    time.sleep(0.01)
    try:
        ser.readline()
        return 1
    except UnicodeDecodeError:
        return 0


def reset_mbed():
    print("Restarting mbed")
    print("Program may exit due to serial reset")
    #Stop thrusters and set Event flags to inhibit new commands.
    stop_persistent_h.set()
    stop_persistent_d.set()
    stop_thrusters_command()
    #Set Event flag to end serial read thread.
    stop_threads.set()
    print("Shutting down thrusters and closing serial link.")
    time.sleep(2)
    ser.send_break()     #break command over serial resets mbed
    writeline=('res:000').encode()
    time.sleep(0.1)
    ser.write(writeline)
    log.info("Raspi command sent: "+writeline.decode())
    print("sent: "+writeline.decode())
    print("Restarting serial link.")
    #clear Event flags.
    stop_threads.clear()
    stop_persistent_h.clear()
    stop_persistent_d.clear()
    i=0
    while (i<100):
        serialreset=0
        j=0
        while (serialreset==0):
            serialreset=clear_serial()
            j+=1
        i+=1
    #Start reading from serial again.
    start_serial_thread()
    return 1

#For error handling
#ensure user input is int or str before using logic
def isInt(string):
    try: 
        int(string)
        return True
    except ValueError:
        return False
        
def isHex(string):
    try:
        int(string,base=16)
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
        print("Select position: [0-7] or [r]eturn to main menu.")
        print("See datasheet for guidance.")
        selection=input(input_string)
        if (selection==stop_command):
            horizon()
            returnval=1
        elif (selection=='r'):
            returnval=1
        elif (isInt(selection)):
            if (int(selection)>=0) and (int(selection)<8):
                writeline=(prefix+"00"+selection).encode()
                ser.write(writeline)
                log.info("Raspi command sent: "+writeline.decode())
                print()
                print("sent: "+writeline.decode())
                time.sleep(1)
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

def depth_zero():
    zero_prefix='zer:'
    writeline=(zero_prefix+"000").encode()
    GPIO.output(40,1)
    ser.write(writeline)
    log.info("Raspi command sent: "+writeline.decode())

# def depth_set():
    # set_prefix='set:'
    # writeline=(set_prefix+"000").encode()
    # ser.write(writeline)
    # log.info("Raspi command sent: "+writeline.decode())

def config_menu():
    selection=0
    while(selection!=1):
        print()
        print("Status: "+str(status))
        print("Config select: [c]alibrate BNO, [p]osition, [o]ffsets, [z]ero depth, [s]et depth cal, [r]eturn for main menu")
        configsel=input(input_string)
        if configsel==stop_command:
            horizon()
            selection=1
        elif configsel=='c':
            selection=set_cal()
        elif configsel=='p':
            selection=set_pos()
        elif configsel=='o':
            selection=set_offsets()
        elif configsel=='z':
            depth_zero()
            time.sleep(1)
            if ((status&0x0008)==0x0008):
                selection=1
                print("Depth values calibrated by mbed.")
#            else:
#                print("Please [s]et depth next to complete mbed depth calibration.")
        # elif configsel=='s':
            # depth_set()
            # time.sleep(1)
            # if ((status&0x0008)==0x0008):
                # selection=1
                # print("Depth values calibrated by mbed.")
            # else:
                # print("Please enter [z]ero depth next to complete mbed depth calibration.")
        elif configsel=='r':
            selection=1
        else:
            selection=0            
            print (error_msg)
        time.sleep(0.005)
    return selection

def reconstruct_pw(data):
    pw=0
    speed_hex=(data & 0xffff)
    speed=(speed_hex/100000)
    if ((data & 0x00010000) == 0x00010000):
        pw=1.5 - speed
    else:
        pw=1.5 + speed
    return pw

def get_angles():
    #Set key values.
    #Keys are first 4 Hexadecimal values in line
    #and used to "tag" what kind of data follows
    ver_key=0x1234
    status_key=0xffff  #also 'ready'
    horizon_key=0xff00
    cal_key=0xc000
    h_key=0xc100
    r_key=0xc300
    p_key=0xc500
    d_key=0xb100
    port_key=0xf100
    starboard_key=0xf200
    fore_key=0xf300
    aft_key=0xf400

    #Set globals accessible outside this thread
    global status
    global calibration
    global heading
    global roll
    global pitch
    global depth
    global horizon_count
    global horizon_state
    global port_pw
    global starboard_pw
    global fore_pw
    global aft_pw

    #Initialize values to 0
    status,calibration,heading,roll,pitch,depth=0,0,0,0,0,0
    port_pw,starboard_pw,fore_pw,aft_pw=0,0,0,0
    horizon_count,horizon_state=0,0
    key,st,cal,h,r,p,d=0x0,0x0,0x0,0x0,0x0,0x0,0x0
    port,star,fore,aft=0,0,0,0
    
    #See mbed code. Roll and Pitch are +/- 180 from BNO
    #Offset allows mbed and raspi to deal strictly with
    #positive numbers.
    #360 deg added on mbed side, must be subtracted
    offset=0x1680

    #mbed sends 8bit words, +1 for overhead
    word_size=9

    #mbed sends 11 8bit words every 20ms
    #Or one word around every 2ms
    #This while loop only processes 1 word at a time
    while not stop_threads.is_set(): 
        #Only proceed if there are bytes in Serial waiting to be read
        while (ser.inWaiting==0):
            time.sleep(0.005)
            print("no serial input... waiting")

        #Read bytes from Serial
        in_buffer=ser.readline()
        
        #Only assign prefix and data if serial line is correct length
        #Prevents Value Errors
        int_buffer_prefix=0x10000
        int_buffer_data=0x10000
        length=len(in_buffer)
        if (length==word_size):
            str_buffer_prefix=in_buffer[0:2].decode()+in_buffer[2:4].decode()
            #print("prefix: "+str(str_buffer_prefix))
            if isHex(str_buffer_prefix):
                int_buffer_prefix=int(str_buffer_prefix,base=16)
            str_buffer_data=in_buffer[4:6].decode()+in_buffer[6:8].decode()
            #print("str prefix: " + str(str_buffer_prefix)+ ", data: "+str(str_buffer_data))
            if isHex(str_buffer_data):
                int_buffer_data=int(str_buffer_data,base=16)
            #print("int prefix: " + str(hex(int_buffer_prefix)) + ", data: " + str(hex(int_buffer_data)))
        
        elif (in_buffer[0:3].decode()=='log'):
            log.info(in_buffer[0:(length-2)].decode())
        else:
            log.debug(in_buffer[0:(length-2)].decode())

        #Sort data to correct variable using the keys
        if (int_buffer_prefix == ver_key):
            key=int_buffer_data
            #print("key detected")
        if (int_buffer_prefix == status_key):
            st=int_buffer_data
            horizon_state=0
            # print("status detected")
        if ((int_buffer_prefix & 0xfff0)==horizon_key):
            st=int_buffer_data
            horizon_count=(int_buffer_prefix & 0x000f)
            horizon_state=1
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
        if (int_buffer_prefix == d_key):
            d=(int_buffer_data/0x20)
        if ((int_buffer_prefix&0xfff0) == port_key):
            port=reconstruct_pw((int_buffer_prefix*0x10000)+int_buffer_data)
        if ((int_buffer_prefix&0xfff0) == starboard_key):
            star=reconstruct_pw((int_buffer_prefix*0x10000)+int_buffer_data)
        if ((int_buffer_prefix&0xfff0) == fore_key):
            fore=reconstruct_pw((int_buffer_prefix*0x10000)+int_buffer_data)
        if ((int_buffer_prefix&0xfff0) == aft_key):
            aft=reconstruct_pw((int_buffer_prefix*0x10000)+int_buffer_data)

#Only update globals after key is verified
        if (key==0xabcd):
            #print("VERIFIED")
            heading = h
            roll = r
            pitch = p
            calibration=cal
            status=st
            depth=d
            port_pw=port
            starboard_pw=star
            fore_pw=fore
            aft_pw=aft
            delimiter=':|:'
            logline=delimiter+str(h)+delimiter+str(r)+delimiter+str(p)+delimiter+str(hex(cal))+delimiter+str(hex(st))+delimiter+str(d)+delimiter+str(port_pw)+delimiter+str(starboard_pw)+delimiter+str(fore_pw)+delimiter+str(aft_pw)
            #reset verifications for next loop
            #print ("cal:" + str(cal) + " heading:"+str(h)+" roll:"+str(r)+" pitch:"+str(p))
            log.info(logline)
            key=0x0
            if  (status & 0x0100) == 0x0100:
                mbed_in_commands.set()
            else:
                mbed_in_commands.clear()
            if (status & 0x0200) == 0x0200:
                mbed_in_call.set()
            else:
                mbed_in_call.clear()
        time.sleep(0.0002)

def stop_thrusters_command():
    writeline=('sto:000').encode()
    ser.write(writeline)
    GPIO.output(40,1)
    print("sent: "+writeline.decode())
    log.info("Raspi command sent: "+writeline.decode())

def roll_command(roll_str):
    target=int(roll_str)
    if (target<10): prefix='rol:00'
    elif (target<100): prefix='rol:0'
    else: prefix='rol:'
    writeline=(prefix+roll_str).encode()
    ser.write(writeline)
    GPIO.output(40,1)
    print("sent: "+writeline.decode())
    log.info("Raspi command sent: "+writeline.decode())

def pitch_command(pitch_str):
    target=int(pitch_str)
    if (target<10): prefix='pit:00'
    elif (target<100): prefix='pit:0'
    else: prefix='pit:'
    writeline=(prefix+pitch_str).encode()
    ser.write(writeline)
    GPIO.output(40,1)
    print("sent: "+writeline.decode())
    log.info("Raspi command sent: "+writeline.decode())
    
def speed_command(speed_str):
    target=int(speed_str)
    if (target<10): prefix='vel:00'
    elif (target<100): prefix='vel:0'
    else: prefix='vel:'
    writeline=(prefix+speed_str).encode()
    ser.write(writeline)
    GPIO.output(40,1)
    print("sent: "+writeline.decode())
    log.info("Raspi command sent: "+writeline.decode())

def heading_command(heading_str):
    target=int(heading_str)
    if (target<10): prefix='hea:00'
    elif (target<100): prefix='hea:0'
    else: prefix='hea:'
    writeline=(prefix+heading_str).encode()
    ser.write(writeline)
    GPIO.output(40,1)
    print("sent: "+writeline.decode())
    log.info("Raspi command sent: "+writeline.decode())

def depth_command(depth_str):
    target=int(depth_str)
    if (target<10): prefix='dep:00'
    elif (target<100): prefix='dep:0'
    else: prefix='dep:'
    writeline=(prefix+depth_str).encode()
    ser.write(writeline)
    GPIO.output(40,1)
    log.info("Raspi command sent: "+writeline.decode())
    print("sent: "+writeline.decode())

def operate(target):
    print("heading: " +str(heading))
    print("calibration: " +str(hex(calibration)) + " status: "+str(hex(status)))
    compare=heading
    success=0

    #Acceptable tolerance in degrees
    #mbed has separate tolerance (likely set tighter)
    tolerance=3
    if (target!=-1):
        heading_command(str(target))
    while (target==-1):
        print()
        print("Heading: " + str(heading) + " Cal: " + str(hex(calibration)))
        print("Enter heading [0 to 360] or [r]eturn to Main Menu")
        decision=input(input_string)
        if (decision==stop_command):
            horizon()
            target=compare
            success=1
        elif (decision=='r'):
            target=compare
            success=1
        elif (isInt(decision)):
            headingselect=int(decision)
            if (headingselect>=0) and (headingselect<=360):
                target=headingselect
                heading_command(decision)
            else: print(error_msg)
        else: print(error_msg)
    while (success!=1):
        stopped=0
        compare=heading
        diff=abs(target-compare)
        if (diff>180):
            if (target>180): diff=((compare+180)-(target-180))
            if (compare>180): diff=((target+180)-(compare-180))
        #print("heading: "+str(heading))
        #compare 3 bits for starboard and port thruster status
        check_thruster_status=(status & 0x00c0)
        if (check_thruster_status==0x0000):
            stopped = 1
            #print("Az thrusters stopped")
        #elif ((starboard_pw<1.5) and (port_pw<1.5)):
            #print("Turning to Port")
        #elif ((starboard_pw>1.5) and (port_pw>1.5)):
            #print("Turning to Starboard")
        #else:
            #print("Unrecognized thruster status")
#        print("diff: " + str(diff) + " heading: " +str(heading))
#        print("cal: " +str(hex(calibration)) + " status: " +str(hex(status)))
#        print()
        if ((diff<=tolerance) and (stopped==1)):
            print("Success.")
            success=1
#        else:
#            heading_command(str(target))
        time.sleep(0.1)
    return target

def set_depth(target):
    print("heading: " +str(heading) + " depth: " + str(depth))
    #Acceptable tolerance in cm
    #mbed has separate tolerance (likely set tighter)
    success=0
    tolerance=1.5;
    if (target!=-1):
        depth_command(str(target))
    while (target==-1):
        print()
        print("Heading: " + str(heading) + " depth: " + str(depth))
        print("Enter depth [0 to 550] or [r]eturn to Main Menu")
        decision=input(input_string)
        if (decision==stop_command):
            horizon()
            target=compare
            success=1
        elif (decision=='r'):
            target=compare
            success=1
        elif (isInt(decision)):
            depthselect=int(decision)
            if (depthselect>=0) and (depthselect<=550):
                target=depthselect
                depth_command(decision)
            else: print(error_msg)
        else: print(error_msg)
    while (success!=1):
        compare=depth
        diff=abs(target-compare)
        time.sleep(0.02)
        #compare 3 bits for starboard and port thruster status
        check_thruster_status=(status & 0x0700)
        if (check_thruster_status==0x0000):
            print("Elevation thrusters stopped")
        elif (check_thruster_status==0x0700):
            print("Going up")
        elif (check_thruster_status==0x0400):
            print("Going down")
        print("depth: "+str(depth))
        print("diff: " + str(diff) + " depth: " +str(depth))
        print()
        if (diff<=tolerance):
            print("Success.")
            success=1
    return target

def depth_thread(target):
    while not stop_persistent_d.is_set():
        #Event "suppress_commands" is used to ensure threaded Heading and Depth commands are spaced out by at least 10ms.
        if not suppress_commands.is_set():
            suppress_commands.set()
            depth_command(str(target))
            #Suppress commands for 10ms.
            time.sleep(0.020)
            #Clear commands.,
            suppress_commands.clear()
            #Sleep for 20ms to 
            time.sleep(0.050)
        #If commands not available, check again in 10ms.
        time.sleep(0.005)

def heading_thread(target):
    while not stop_persistent_h.is_set():
        #Event "suppress_commands" is used to ensure threaded Heading and Depth commands are spaced out by at least 10ms.
        if not suppress_commands.is_set():
            suppress_commands.set()
            heading_command(str(target))
            #Suppress commands for 20ms.
            time.sleep(0.020)
            #Clear commands.,
            suppress_commands.clear()
            #Sleep for 50ms
            time.sleep(0.050)
        #If commands not available, check again in x ms.
        time.sleep(0.005)

def start_threads(type,value):
    returnval=0
    if type=='d':
#        print("Starting persistent depth thread")
#        persistent_depth_thread=threading.Thread(target=depth_thread,args=(value,))
#        persistent_depth_thread.start()
        depth_thread=threading.Thread(target=set_depth,args=(value,))
        returnval=1
    if type=='h':
#        print("Starting persistent heading thread")
#        persistent_heading_thread=threading.Thread(target=heading_thread,args=(value,))
#        persistent_heading_thread.start()
        heading_thread=threading.Thread(target=operate,args=(value,))
        heading_thread.start()
        returnval=1
    return returnval

def direction_command(i):
    if (i<10): prefix='tst:00'
    elif (i<100): prefix='tst:0'
    else: prefix='tst:'
    writeline=(prefix+str(i)).encode()
    ser.write(writeline)
    log.info("Raspi command sent: "+writeline.decode())


def test_mode():
    quit=0
    while (quit!=1):
        print()
        print("select [m]ove forwards and backwards, [t]urn left and right, [e]levation up and down, ")
        print("[p]itch front and back, or [q]uit to main menu")
        codesel=input(input_string)
        if (codesel==stop_command):
            horizon()
            stop_thrusters_command()
            quit=1
        elif (codesel=='q'):
            stop_thrusters_command()
            quit=1
        elif (codesel=='m'):
            print("moving forwards and backwards for a few seconds")
            direction_command(1)
        elif (codesel=='t'):
            print("turning left and right for a few seconds")
            direction_command(2)
        elif (codesel=='e'):
            print("going up and down for a few seconds")
            direction_command(3)
        elif (codesel=='p'):
            print("pitching forward and reverse for a few seconds")
            direction_command(4)
        else:
            print(error_msg)
    return quit

def start_serial_thread():
    ReadSerial_thread=threading.Thread(target=get_angles,args=())
    ReadSerial_thread.start()

def check_gui_speed(v):
    if (isInt(v)):
        speed=int(v)
        if (speed>=-400) and (speed<=400):
            speed+=400
            speed_command(str(speed))
        else:
            print("Invalid GUI speed")
    else:
        print("GUI speed not integer")

def check_gui_heading(h):
    if (isInt(h)):
        heading=int(h)
        if (heading>=0) and (heading<=360):
            #stop_persistent_h.set()
            #time.sleep(0.05)
            #stop_persistent_h.clear()
            #start_threads(h,heading)
            #operate(heading)
            heading_command(h)
        else:
            print("Invalid GUI heading")
    else:
        print("GUI heading not integer")

def check_gui_roll(r):
    if (isInt(r)):
        roll=int(r)
        if (roll>=-180) and (roll<=180):
            roll+=180
            roll_command(str(roll))
        else:
            print("Invalid GUI roll")
    else:
        print("GUI roll not integer")

def check_gui_pitch(p):
    if (isInt(p)):
        pitch=int(p)
        if (pitch>=-180) and (pitch<=180):
            pitch=pitch+180
            pitch_command(str(pitch))
        else:
            print("GUI pitch not integer")
    else:
        print("GUI pitch not integer")

def check_gui_depth(d):
    if ((status&0x0008)!=0x0008):
        print("Depth zero and calibration not set")
    if (isInt(d)):
        #depth=int(d)
        #stop_persistent_d.set()
        #time.sleep(0.05)
        #stop_persistent_d.clear()
        #start_threads(d,depth)
        #set_depth(depth)
        if ((int(d)<=820) and (int(d)>=0)):
            depth_command(d)
    else:
        print("GUI depth not integer")
#    else:
#        print("Depth zero and calibration not set")

class Application(Frame):

    def __init__(self, master):
        Frame.__init__(self, master)
        self.pack()
        self.widgets()
        self.update_values()
        
    def widgets(self):
        
        self.heading_gui_data=IntVar()
        self.roll_gui_data=IntVar()
        self.pitch_gui_data=IntVar()
        self.depth_gui_data=IntVar()
        self.port_gui_data=IntVar()
        self.star_gui_data=IntVar()
        self.fore_gui_data=IntVar()
        self.aft_gui_data=IntVar()
        self.cal_gui_data=StringVar()
        
        self.heading_text = Label(self,text="Heading:")
        self.heading_text.grid(column=1,row=1)
        self.heading_gui = Label(self,textvariable=self.heading_gui_data)
        self.heading_gui.grid(column=2,row=1)

        self.roll_text = Label(self,text="Roll:")
        self.roll_text.grid(column=1,row=2)
        self.roll_gui = Label(self,textvariable=self.roll_gui_data)
        self.roll_gui.grid(column=2,row=2)

        self.pitch_text = Label(self,text="Pitch:")
        self.pitch_text.grid(column=1,row=3)
        self.pitch_gui = Label(self,textvariable=self.pitch_gui_data)
        self.pitch_gui.grid(column=2,row=3)

        self.depth_text = Label(self,text="Depth:")
        self.depth_text.grid(column=1,row=4)
        self.depth_gui = Label(self,textvariable=self.depth_gui_data)
        self.depth_gui.grid(column=2,row=4)

        self.port_pw_text=Label(self,text="Port pw:")
        self.port_pw_text.grid(column=1,row=5)
        self.port_gui = Label(self,textvariable=self.port_gui_data)
        self.port_gui.grid(column=2,row=5)

        self.star_pw_text=Label(self,text="Stbd pw:")
        self.star_pw_text.grid(column=1,row=6)
        self.star_gui=Label(self,textvariable=self.star_gui_data)
        self.star_gui.grid(column=2,row=6)

        self.fore_pw_text=Label(self,text="Fore pw:")
        self.fore_pw_text.grid(column=1,row=7)
        self.fore_gui=Label(self,textvariable=self.fore_gui_data)
        self.fore_gui.grid(column=2,row=7)

        self.aft_pw_text=Label(self,text="Aft pw:")
        self.aft_pw_text.grid(column=1,row=8)
        self.aft_gui=Label(self,textvariable=self.aft_gui_data)
        self.aft_gui.grid(column=2,row=8)
        
        self.BNO_status_text=Label(self,text="BNO status:")
        self.BNO_status_text.grid(column=1,row=9)
        self.BNO_status_gui=Label(self,textvariable=self.cal_gui_data)
        self.BNO_status_gui.grid(column=2,row=9)

        self.heading_field_text=Label(self,text="Enter heading:")
        self.heading_field_text.grid(column=3,row=1)
        self.heading_field=Entry(self)
        self.heading_field.grid(column=4,row=1)
        self.heading_field_button = Button(self,text="Go to heading", command=lambda: check_gui_heading(self.heading_field.get()))
        self.heading_field_button.grid(column=5,row=1)
        self.heading_field_stop = Button(self,text="Stop heading", command=lambda: heading_command(str(999)))
        self.heading_field_stop.grid(column=6,row=1)
        
        self.speed_field_text=Label(self,text="Enter roll:")
        self.speed_field_text.grid(column=3,row=2)
        self.speed_field=Entry(self)
        self.speed_field.grid(column=4,row=2)
        self.speed_field_button = Button(self,text="Go to roll", command=lambda: check_gui_speed(self.speed_field.get()))
        self.speed_field_button.grid(column=5,row=2)
        self.speed_field_stop = Button(self,text="Stop roll", command=lambda: speed_command(str(999)))
        self.speed_field_stop.grid(column=6,row=2)

        self.pitch_field_text=Label(self,text="Enter pitch:")
        self.pitch_field_text.grid(column=3,row=3)
        self.pitch_field=Entry(self)
        self.pitch_field.grid(column=4,row=3)
        self.pitch_field_button = Button(self,text="Go to pitch", command=lambda: check_gui_pitch(self.pitch_field.get()))
        self.pitch_field_button.grid(column=5,row=3)
        self.pitch_field_stop = Button(self,text="Stop pitch", command=lambda: pitch_command(str(999)))
        self.pitch_field_stop.grid(column=6,row=3)

        self.depth_field_text=Label(self,text="Enter depth:")
        self.depth_field_text.grid(column=3,row=4)
        self.depth_field=Entry(self)
        self.depth_field.grid(column=4,row=4)
        self.depth_set_indicator=Label(self,text="Depth Zeroized",bg="white")
        self.depth_set_indicator.grid(column=4,row=5)
        self.depth_field_button = Button(self,text="Go to depth",command=lambda: check_gui_depth(self.depth_field.get()))
        self.depth_field_button.grid(column=5,row=4)
        self.depth_zero_button = Button(self,text="Depth zero",command=lambda: depth_zero())
        self.depth_zero_button.grid(column=5,row=5)
        self.depth_field_stop = Button(self,text="Stop depth", command=lambda: depth_command(str(999)))
        self.depth_field_stop.grid(column=6,row=4)
        
        self.speed_plus_button = Button(self,text="Speed Up",command=lambda: speed_command(str(847)))
        self.speed_plus_button.grid(column=5,row=6)
        self.speed_plus_button = Button(self, text="Speed Down",command=lambda: speed_command(str(841)))
        self.speed_plus_button.grid(column=6, row=6)
        
        self.speed_plus_button = Button(self,text="Heading Up",command=lambda: heading_command(str(837)))
        self.speed_plus_button.grid(column=5,row=7)
        self.speed_plus_button = Button(self, text="Heading Down",command=lambda: heading_command(str(831)))
        self.speed_plus_button.grid(column=6, row=7)
        self.speed_plus_button = Button(self,text="Depth Up",command=lambda: depth_command(str(821)))
        self.speed_plus_button.grid(column=5,row=8)
        self.speed_plus_button = Button(self, text="Depth Down",command=lambda: depth_command(str(824)))
        self.speed_plus_button.grid(column=6, row=8)
        self.speed_plus_button = Button(self,text="Pitch Up",command=lambda: pitch_command(str(817)))
        self.speed_plus_button.grid(column=5,row=9)
        self.speed_plus_button = Button(self, text="Pitch Down",command=lambda: pitch_command(str(811)))
        self.speed_plus_button.grid(column=6, row=9)

        self.Horizon_button = Button(self,text="EMERGENCY STOP",command=lambda: horizon())
        self.Horizon_button.grid(column=2,row=0)

        self.command_indicator = Label(self,text="Command Function",bg="white")
        self.command_indicator.grid(row=0,column=5)
        self.logic_indicator = Label(self,text="Logic Function",bg="white")
        self.logic_indicator.grid(row=0,column=6)
        self.call_indicator = Label(self,text="Call Function",bg="white")
        self.call_indicator.grid(row=0,column=7)

        self.Test1_button = Button(self,text="Test fwd/back",command=lambda: direction_command(1))
        self.Test1_button.grid(column=8,row=1)
        self.Test2_button = Button(self,text="Test left/right",command=lambda: direction_command(2))
        self.Test2_button.grid(column=8,row=2)
        self.Test3_button = Button(self,text="Test up/down",command=lambda: direction_command(3))
        self.Test3_button.grid(column=8,row=3)
        self.Test4_button = Button(self,text="Test pitch",command=lambda: direction_command(4))
        self.Test4_button.grid(column=8,row=4)

        self.Quit_button=Button(self,text="Leave GUI",command=lambda: self.quit_method())
        self.Quit_button.grid(column=8,row=0)
        
        self.reset_button=Button(self,text="Reset Mbed",command=lambda: reset_mbed())
        self.reset_button.grid(column=0,row=0)

    def quit_method(self):
        quit_window.set()
        self.quit
    def heading(self,h):
        self.heading_gui_data.set(h)
    def roll(self,r):
        self.roll_gui_data.set(r)
    def pitch(self,p):
        self.pitch_gui_data.set(p)
    def depth(self,d):
        self.depth_gui_data.set(d)
    def port(self,pw):
        self.port_gui_data.set(pw)
    def star(self,pw):
        self.star_gui_data.set(pw)
    def fore(self,pw):
        self.fore_gui_data.set(pw)
    def aft(self,pw):
        self.aft_gui_data.set(pw)
    def cal(self,cal_data):
        self.cal_gui_data.set(str(hex(cal_data)))
    def change_green(self,word):
        if word=='depth':
            self.depth_set_indicator.config(bg="green2")
        if word=='command':
            self.command_indicator.config(bg="green2")
        if word=='call':
            self.call_indicator.config(bg="green2")
        if word=='logic':
            self.logic_indicator.config(bg="green2")
    def change_white(self,word):
        if word=='depth':
            self.depth_set_indicator.config(bg="white")
        if word=='command':
            self.command_indicator.config(bg="white")
        if word=='call':
            self.call_indicator.config(bg="white")
        if word=='logic':
            self.logic_indicator.config(bg="white")

    def update_values(self):
        self.heading(heading)
        self.roll(roll)
        self.pitch(pitch)
        self.depth(depth)
        self.port(port_pw)
        self.star(starboard_pw)
        self.fore(fore_pw)
        self.aft(aft_pw)
        self.cal(calibration)
        if ((status&0x0008)==0x0008):
            self.change_green('depth')
        else:
            self.change_white('depth')
        if ((status&0x0100)==0x0100):
            self.change_green('command')
        else:
            self.change_white('command')
        if ((status&0x0200)==0x0200):
            self.change_green('call')
        else:
            self.change_white('call')       
        if ((status&0x0800)==0x0800):
            self.change_green('logic')
        else:
            self.change_white('logic')
        self.update()
        self.update_idletasks()

def windowsetup():
        root = Tk()
        root.title("UAV Controls")
        root.geometry('1100x400')
        app = Application(master=root)
        #app.mainloop()
        i=0
        while not quit_window.is_set():
        #while(1):
        #print('while loop')
                app.update_values()
                time.sleep(0.1)
        root.destroy()
        quit_window.clear()
   
def start_window_thread():
    Window_thread=threading.Thread(target=windowsetup,args=())
    Window_thread.start()

#Main menu
#outsource selection to individual functions
def main():
    #Open separate thread for intaking Serial data stream from mbed
    start_serial_thread()
    print()
    print("Global Commands: ")
    print("'STOP' will put mbed in an emergency state and shut off all thrusters")
    quit=0
    persistent_h=0
    persistent_d=0
    logline='KEY: heading:|:roll:|:pitch:|:BNO cal:|:status:|:depth:|:port pw:|:starboard pw:|:fore pw:|:aft pw'
    log.info(logline)
    count=9000
    count=0
    while (quit!=1):
        if (count>1000):
            quit=1
        time.sleep(0.03)
        GPIO.output(40,0)
        time.sleep(0.07)
        heading_command(str(999))
        count+=1
    stop_thrusters_command()
    stop_persistent_h.set()
    stop_persistent_d.set()
    GPIO.cleanup()
    try:
        quit_window.set()
        stop_threads.set()
        time.sleep(1)
        return 1
    except:
        pass

main()

