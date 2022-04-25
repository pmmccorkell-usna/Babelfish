import time
import serial
import threading
import logging
import logging.handlers
from datetime import datetime

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
filename = datetime.now().strftime('./log/AUV_%Y%m%d_%H:%M:%s.log')
log = logging.getLogger()
log.setLevel(logging.INFO)
format = logging.Formatter('%(asctime)s : %(message)s')
file_handler = logging.FileHandler(filename)
file_handler.setLevel(logging.INFO)
file_handler.setFormatter(format)
log.addHandler(file_handler)


#Set global killswitch event across threads
stop_threads=threading.Event()
stop_persistent_h=threading.Event()
stop_persistent_d=threading.Event()

#lazy ... dont want to change these everywhere
input_string=">> "
error_msg="Invalid entry. Try again."
return_menu="Returning to main menu."
stop_command="STOP"

def horizon():
    writeline=stop_command.encode()
    print("sent: "+writeline.decode())
    ser.write(writeline)
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
        success=reset_mbed()

def reset_mbed():
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
            ser.send_break      #break command over serial resets mbed
            writeline=('res:000').encode()
            ser.write(writeline)
            print("sent: "+writeline.decode())
            print("Restarting serial link.")
            #clear Event flags.
            stop_threads.clear()
            stop_persistent_h.clear()
            stop_persistent_d.clear()
            time.sleep(2)
            #Start reading from serial again.
            start_serial_thread()
            reset=1
            success=1
        elif (selection=='n'):
            success=1
        else:
            print(error_msg)
    return reset

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
                print()
                print("sent: "+writeline.decode())
                ser.write(writeline)
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
            zero_prefix='zer:'
            writeline=(prefix+"000").encode()
            ser.write(writeline)
            time.sleep(1)
            if ((status&0x0008)==0x0008):
                selection=1
                print("Depth values calibrated by mbed.")
            else:
                print("Please [s]et depth next to complete mbed depth calibration.")
        elif configsel=='s':
            set_prefix='set:'
            writeline=(prefix+"000").encode()
            ser.write(writeline)
            time.sleep(1)
            if ((status&0x0008)==0x0008):
                selection=1
                print("Depth values calibrated by mbed.")
            else:
                print("Please enter [z]ero depth next to complete mbed depth calibration.")
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
    horizon_key=0xff00
    cal_key=0xc000
    h_key=0xc100
    r_key=0xc300
    p_key=0xc500
    d_key=0xb100

    #Set globals accessible outside this thread
    global status
    global calibration
    global heading
    global roll
    global pitch
    global depth
    global horizon_count
    global horizon_state

    #Initialize values to 0
    status,calibration,heading,roll,pitch,depth=0,0,0,0,0,0
    horizon_count,horizon_state=0,0
    key,st,cal,h,r,p,d=0x0,0x0,0x0,0x0,0x0,0x0,0x0
    
    #See mbed code. Roll and Pitch are +/- 180 from BNO
    #Offset allows mbed and raspi to deal strictly with
    #positive numbers.
    #360 deg added on mbed side, must be subtracted
    offset=0x1680

    #mbed sends 8bit words, +1 for overhead
    word_size=9

    #mbed sends 6 8bit words every 20ms
    #Or one word around every 3ms
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
        if (len(in_buffer)==word_size):
            str_buffer_prefix=in_buffer[0:2].decode()+in_buffer[2:4].decode()
            #print("prefix: "+str(str_buffer_prefix))
            if isHex(str_buffer_prefix):
                int_buffer_prefix=int(str_buffer_prefix,base=16)
            str_buffer_data=in_buffer[4:6].decode()+in_buffer[6:8].decode()
            #print("str prefix: " + str(str_buffer_prefix)+ ", data: "+str(str_buffer_data))
            if isHex(str_buffer_data):
                int_buffer_data=int(str_buffer_data,base=16)
            #print("int prefix: " + str(hex(int_buffer_prefix)) + ", data: " + str(hex(int_buffer_data)))

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
            d=(int_buffer_data/1000)
#Only update globals after key is verified
        if (key==0xabcd):
            #print("VERIFIED")
            heading = h
            roll = r
            pitch = p
            calibration=cal
            status=st
            depth=d
            delimiter=':|:'
            logline=delimiter+str(h)+delimiter+str(r)+delimiter+str(p)+delimiter+str(hex(cal))+delimiter+str(hex(st))+delimiter+str(d)
            #reset verifications for next loop
            #print ("cal:" + str(cal) + " heading:"+str(h)+" roll:"+str(r)+" pitch:"+str(p))
            log.info(logline)
            key=0x0
        time.sleep(0.0003)
        
def stop_thrusters_command():
    writeline=('sto:000').encode()
    ser.write(writeline)

def Heading_command(heading_str):
    target=int(heading_str)
    if (target<10): prefix='hea:00'
    elif (target<100): prefix='hea:0'
    else: prefix='hea:'
    writeline=(prefix+heading_str).encode()
#    print("sent: "+writeline.decode())
    ser.write(writeline)

def operate(target):
    print("heading: " +str(heading))
    print("calibration: " +str(hex(calibration)) + " status: "+str(hex(status)))
    compare=heading
    success=0

    #Acceptable tolerance in degrees
    #mbed has separate tolerance (likely set tighter)
    tolerance=3
    if (target!=-1):
        Heading_command(str(target))
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
                Heading_command(decision)
            else: print(error_msg)
        else: print(error_msg)
    while (success!=1):
        stopped=0
        compare=heading
        diff=abs(target-compare)
        if (diff>180):
            if (target>180): diff=((compare+180)-(target-180))
            if (compare>180): diff=((target+180)-(compare-180))
        print("heading: "+str(heading))
        #compare 3 bits for starboard and port thruster status
        check_thruster_status=(status & 0x0070)
        if (check_thruster_status==0x0000):
            stopped = 1
            print("Az thrusters stopped")
        elif (check_thruster_status==0x0050):
            print("Turning to Starboard")
        elif (check_thruster_status==0x0060):
            print("Turning to Port")
        else:
            print("Unrecognized thruster status")
#        print("diff: " + str(diff) + " heading: " +str(heading))
#        print("cal: " +str(hex(calibration)) + " status: " +str(hex(status)))
#        print()
        if ((diff<=tolerance) and (stopped==1)):
            print("Success.")
            success=1
#        else:
#            Heading_command(str(target))
        time.sleep(0.1)
    return target

def depth_command(depth_str):
    target=int(depth_str)
    if (target<10): prefix='dep:00'
    elif (target<100): prefix='dep:0'
    else: prefix='dep:'
    writeline=(prefix+depth_str).encode()
    print("sent: "+writeline.decode())
    ser.write(writeline)

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
        depth_command(str(target))
        time.sleep(0.025)

def heading_thread(target):
    while not stop_persistent_h.is_set():
        Heading_command(str(target))
        time.sleep(0.025)

def start_threads(type,value):
    returnval=0
    if type=='d':
        print("Starting persistent depth thread")
        persistent_depth_thread=thread.Thread(target=depth_thread,args=(value,))
        persistent_depth_thread.start()
        returnval=1
    if type=='h':
        print("Starting persistent heading thread")
        persistent_heading_thread=threading.Thread(target=heading_thread,args=(value,))
        persistent_heading_thread.start()
        returnval=1
    return returnval

def direction_command(i):
    if (i<10): prefix='tst:00'
    elif (i<100): prefix='tst:0'
    else: prefix='tst:'
    writeline=(prefix+str(i)).encode()
#    print("sent: "+writeline.decode())
    ser.write(writeline)

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
    logline='KEY: heading:|:roll:|:pitch:|:BNO cal:|:status:|:depth'
    log.info(logline)
    while (quit!=1):
        print()
        print("Mode select: [c]onfig, [t]est mode, [m]onitor data, [d]epth, [g]o to heading, ")
        print("[p]ersistent heading, [r]andom heading, [q]uit")
        modesel=input(input_string)
        if modesel==stop_command:
            horizon()
            quit=1
        elif modesel=='c':
            result=config_menu()
            if (result):
                print(return_menu)
        elif modesel=='t':
            result=test_mode()
            if (result):
                print(return_menu)
        elif modesel=='m':
            while 1:
                print("VERIFIED HEADING: " + str(heading) + " Cal: " +str(hex(calibration)))
                print("roll: "+str(roll)+" pitch: "+str(pitch) + " depth: "+str(depth))
                print("Status: " + str(hex(status)))
                time.sleep(0.02)
        elif modesel=='d':
            #Ensure zero and cal have been set before continuing.
            if ((status&0x0008)!=0x0008):
                print("Error: Depth zero and/or cal not set. Set through [c]onfig menu.")
                print(return_menu)
            elif (persistent_d==0):
                d=set_depth(-1)
                time.sleep(0.05)
                persistent_d=start_threads('d',d)
            elif (persistent_d==1):
                print("CAUTION: Stopping persistent depth")
                stop_persistent_d.set()
                time.sleep(0.05)
                stop_persistent_d.clear()
                d=set_depth(-1)
                time.sleep(0.05)
                persistent_d=start_threads('d',d)
        elif modesel=='g':
            if(persistent_h==1):
                print("CAUTION: Stopping persistent heading")
                stop_persistent_h.set()
                time.sleep(0.05)
                persistent_h=0;
                stop_persistent_h.clear()
            result=operate(-1)
            if (result):
                print(return_menu)
        elif modesel=='reset':
            reset_mbed()
            print(return_menu)
        elif modesel=='p':
            if (persistent_h==0):
                h=operate(-1)
                time.sleep(0.05)
                persistent_h=start_threads('h',h)
            elif (persistent_h==1):
                print("CAUTION: Stopping persistent heading")
                #set event to end current heading thread
                stop_persistent_h.set()
                time.sleep(0.05)
                #reset event after heading thread has been joined
                stop_persistent_h.clear()
                h=operate(-1)
                time.sleep(0.05)
                persistent_h=start_threads('h',h)
        elif modesel=='r':
            if(persistent_h==1):
                print("CAUTION: Stopping persistent heading")
                stop_persistent_h.set()
                time.sleep(0.05)
                persistent_h=0;
                stop_persistent_h.clear()
            while(1):
                h=random.randint(0,360)
                operate(h)
                time.sleep(0.02)
        elif modesel=='q':
            quit=1
        else:
            print(error_msg)
    stop_thrusters_command()
    stop_persistent_h.set()
    stop_persistent_d.set()
    stop_threads.set()
    time.sleep(1)
    return 1



main()





