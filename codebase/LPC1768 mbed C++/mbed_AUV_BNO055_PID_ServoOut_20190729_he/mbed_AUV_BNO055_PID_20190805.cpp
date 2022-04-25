// AUV test program for reading IMU and depth data and controlling vehicle state
// J. Bradshaw 20190723
//  8-1-2019: Tested in Hydro in the 120" tank on 8-12019.  Dialed in gains approximately.
//   Added tickerReadSensors to fix scanf() freeze issue.
//  8-2-2019: Fixed Depthzero(), added logging with uSD, added options for output stream
//   formatting (JOSN, CSV, terminal interface

#include "mbed.h"
#include "BNO055.h"
#include "PID.h"
#include "RunTimer.h"
#include "ServoOut.h"
#include "MS5837_test.h"
#include "SDFileSystem.h"       //File system for SD Card

#define MODSERIAL_DEFAULT_RX_BUFFER_SIZE 1024
#define MODSERIAL_DEFAULT_TX_BUFFER_SIZE 1024

#define VERSION_STRING  "v0.12"
#include "MODSERIAL.h"
#define PI 3.1415926545359f

#define PITCH_MIN_SP    (-1.5f)         // Min Pitch in radians
#define PITCH_MAX_SP    (1.5)           // Max Pitch in radians

//add sufficient wrap around for the controller to work
#define YAW_MIN_SP      (-3.0*PI)          // Min Yaw in radians   
#define YAW_MAX_SP      (3.0 * PI)      // Max Yaw in radians
#define YAW640_RATIO    (.981746f)      //for some reason, the YAW RADIAN measurement from the BNO
                                        // yeilds 0.0 - 6.40 instead of 2*PI, this scales it accordingly
#define DEPTH_MIN_SP    (0.0f)          // Min Depth in meters
#define DEPTH_MAX_SP    (3.0)           // Max Depth in meters

#define THRUST_MIN_SP   (-0.6)           //Min Thrust value (currently ESC normalized val from -1.0 to 1.0)
#define THRUST_MAX_SP   (0.6)            //Max Thrust value (currently ESC normalized val from -1.0 to 1.0)

// Terminal command sequences.
#define CLR_SCR       "\33[;H\33[2J"
#define MOVE_L_C      "\33[%d;%dH\33[K"
#define RED_FONT      "\33[0;31m"
#define BLUE_FONT     "\33[0;34m"
#define GREEN_FONT    "\33[0;32m"
#define YELLOW_FONT   "\33[0;33m"
#define WHITE_FONT    "\e[97m"
#define DEFAULT_FONT  "\33[0m"
#define ERASE_LINE    "\33[K"

//Instantiate classes
MODSERIAL pc(USBTX, USBRX);
SDFileSystem sd(p5, p6, p7, p8, "sd");               // MOSI, MISO, SCLK, CS
BNO055 bno(p28, p27);   // (SDA, SCL) or... SDA is p28, SCL is p27
MS5837 depthsensor(p28, p27, ms5837_addr_no_CS);
PID pid_pitch(0.0, 0.0, 0.0, .1);
PID pid_yaw(0.0, 0.0, 0.0, .1);
PID pid_depth(0.0, 0.0, 0.0, .1);

Ticker tickerPulse;                 //gives the mbed's LED1 a heartbeat so you know it's alive
Ticker tickerReadSensors;           //ticker to read the on-board sensors (BNO-055 IMU, Pressure sensor)

//Run Timer for total runtime (regular timer class wraps too early)
RunTimer runTime;

PwmOut led_pulse(LED1);
DigitalOut led_sort(LED2);
DigitalOut led_log(LED4);

ServoOut m1_servoOut(p24);     //Top Fore
ServoOut m2_servoOut(p25);     //Top Aft
ServoOut m3_servoOut(p22);     //starboard thruster
ServoOut m4_servoOut(p21);     //port thruster
float escsign_m1 = -1.0;
float escsign_m2 = 1.0;
float escsign_m3 = -1.0;
float escsign_m4 = -1.0;

//pitch controller global variables
Ticker tickerPitch;
int tickerPitchRunning=0;
volatile float pitch_Tdelay = .02;
volatile float pitch_Pk =4.0f;
volatile float pitch_Ik =0.0;   //0.025f;
volatile float pitch_Dk =0.00001;
volatile float pitch_sp = 0.0;      //initialize at 0.0 rad position pitch
volatile float pitch_co = 0.0;
volatile float pitch_sign = 1.0;    // should be +1.0 or -1.0

//yaw/heading controller global variables
Ticker tickerYaw;
int tickerYawRunning=0;
volatile float yaw_Tdelay = .02;
volatile float yaw_Pk =20.5f;
volatile float yaw_Ik =1.5f;      //.055
volatile float yaw_Dk =0.00005;
volatile float yaw_sp = 0.0;      //initialize at 0.0 rad position pitch
volatile float yaw_co = 0.0;
volatile float yaw_sign_calc=0.0;           //used during direction calculation difference
volatile float yaw_sign=1.0;                //user programmable sign applied to output actuators
float yaw_error=0.0;     //make global for now for testing purposes
float yaw_err, yaw_P, yaw_I, yaw_D;

float yaw_temp=0.0;     //make global for now for testing purposes
float yaw_cor=0.0;      //corrected yaw heading in radians

volatile float yaw_temp_error=0.0;
volatile float yaw_temp_heading = 0.0;

//depth controller global variables
Ticker tickerDepth;
int tickerDepthRunning=0;
volatile float depth_m=0.0;
volatile float depth_Tdelay = .1;       //100 millisecond update
volatile float depth_Pk =10.0f;
volatile float depth_Ik =0.0f;      //
volatile float depth_Dk =0.0;
volatile float depth_sp = 0.0;      //initialize at 0.0 meters of depth
volatile float depth_co = 0.0;
volatile float depth_sign = 1.0;    // should be +1.0 or -1.0
volatile float depth_offset = 0.0;    // calculate the zero offset

// Eventually make a controller for the thrust, but for now
//  just use this variable to add to he thruster ESC's (-1.0 to 1.0)
float thrust = 0.0;                 // -1.0 to 1.0 output for reversible ESC's
float thrust_sign = -1.0;            // Allows the user can change the sign of the                                     // thrust output manually

//main function that updates the controller outputs and drives actuators
Ticker tickerUpdateActuators;
int tickerUpdateActuatorsRunning=0;
float updateActuators_Tdelay = .02;

char calib_local[] = {0x0A,0x00,
                        0x19,0x00,
                        0x07,0x00,
                        0x40,0x00,
                        0x5F,0x00,
                        0xE8,0x01,
                        0x01,0x00,
                        0x00,0x00,
                        0x00,0x00,
                        0xE8,0x03,
                        0xFF,0x02};                      

int log_flag = 0;   //global variable for logging data
float log_sampTime = .05;
float log_nextSamp = 0.0;
int stream_flag = 1;
int serOutputMode = 2;

//----- Prototypes ------------------------
void pitchController(void);
void yawController(void);
void depthController(void);
void heartbeat(void);
void driveESCrev(int esc, float driveVal);
void bno_init(void);
void display_commands(void);
void init_pitchController(void);
void init_yawController(void);
void init_depthController(void);
void updateActuators(void);


//-------- FUNCTIONS ----------------------                        
void pitchController(void){
    //set the set point
    pid_pitch.setSetPoint(pitch_sp);
    
    //Update the process variable.
    pid_pitch.setProcessValue(bno.euler.pitch);
            
    //Set the new output.
    pitch_co = pid_pitch.compute();    
}

int con_state;
float yaw_err_unwrapped;
//-------- FUNCTIONS ----------------------                        
void yawController(void){
 //first check for the sign, which direction is faster to turn in
    
    //set the set point
    pid_yaw.setSetPoint(yaw_sp);
    
    yaw_err_unwrapped = yaw_sp - yaw_cor;
    
    if(yaw_sp >= yaw_cor){                   //If the set point is greater then the corrected heading
        yaw_error = yaw_sp - yaw_cor;       //get the difference
        
        if(yaw_error <= PI){        //Turn left
            con_state = 1;
            yaw_sign_calc=1.0; 
            
            //is the error < -PI
            if((yaw_error < 0.0) && (fabs(yaw_error) > PI))
              yaw_error += 2.0*PI;
              
             //is the error < -PI
            if((yaw_error > 0.0) && (fabs(yaw_error) > PI))
              yaw_error = 2.0*PI - yaw_error;
            
            //calculate the heading offset from error relative to setpoint for controller  
            if(yaw_error < 0.0)
                yaw_temp_error = yaw_sp + yaw_error;
            else
                yaw_temp_error = yaw_sp - yaw_error;       
        }                  
        else if(yaw_error > PI){       //Turn right
            con_state = 2;
            yaw_sign_calc=-1.0;
            
            //is the error < -PI
            if((yaw_error < 0.0) && (fabs(yaw_error) > PI))
              yaw_error += 2.0*PI;
              
             //is the error < -PI
            if((yaw_error > 0.0) && (fabs(yaw_error) > PI))
              yaw_error = 2.0*PI - yaw_error;
            
            //calculate the heading offset from error relative to setpoint for controller  
            if(yaw_error < 0.0)
                yaw_temp_error = yaw_sp - yaw_error;
            else
                yaw_temp_error = yaw_sp + yaw_error; 
        }
    
        //Update the process variable.
        pid_yaw.setProcessValue(yaw_temp_error);
    }    
    else if(yaw_sp < yaw_cor){
        yaw_error = yaw_cor - yaw_sp;
        if(yaw_error <= PI){    //difference is
            con_state = 3; 
            yaw_sign_calc=-1.0;              //Turn left

            //is the error < -PI
            if((yaw_error < 0.0) && (fabs(yaw_error) > PI))
              yaw_error += 2.0*PI;
                            //is the error < -PI
            if((yaw_error > 0.0) && (fabs(yaw_error) > PI))
              yaw_error = 2.0*PI - yaw_error;
            
            //calculate the heading offset from error relative to setpoint for controller  
            if(yaw_error < 0.0)
                yaw_temp_error = yaw_sp - yaw_error;
            else
                yaw_temp_error = yaw_sp + yaw_error;                    
        }
        else if(yaw_error > PI){   //180
            con_state = 4;
            yaw_sign_calc=1.0;           //turn right
            //is the error < -PI
            if((yaw_error < 0.0) && (fabs(yaw_error) > PI))
              yaw_error += 2.0*PI;
                            //is the error < -PI
            if((yaw_error > 0.0) && (fabs(yaw_error) > PI))
              yaw_error = 2.0*PI - yaw_error;
            
            //calculate the heading offset from error relative to setpoint for controller  
            if(yaw_error < 0.0)
                yaw_temp_error = yaw_sp + yaw_error;
            else
                yaw_temp_error = yaw_sp - yaw_error;
        }
        //Update the process variable.
        pid_yaw.setProcessValue(yaw_temp_error);
    }
        
    //Set the new output.
    yaw_co = pid_yaw.compute();        
}

void depthController(void){
    //read the depth sensor
    //depthsensor.Barometer_MS5837();    
    
    //set the set point
    pid_depth.setSetPoint(depth_sp);
    
    //Update the process variable.
    pid_depth.setProcessValue(depth_m);
            
    //Set the new output.
    depth_co = pid_depth.compute();
        
/*
    printf("Pressure: "); 
    printf("%f",sensor.MS5837_Pressure()); 
    printf(" mbar");
      
    printf("Temperature: "); 
    printf("%f",sensor.MS5837_Temperature()); 
    printf(" deg C ");
      
    printf("Depth: "); 
    printf("%f",sensor.depth()); 
    printf(" m ");
      
    printf("Altitude: "); 
    printf("%f",sensor.altitude()); 
    printf(" m above mean sea level\r\n");
*/
}

//datasheet specifies 17.2ms MAX at 8192
// resolution for 24-bit sigma-delta analog-to-digital converter
// (or "Fast conversion down to 0.5ms" at 256 bit resolution)
void read_depthSensor(void){
    depthsensor.Barometer_MS5837();    
}

//combines the controller outputs to control actuators
void updateActuators(void){
    //Update the actuator outputs (reversible ESC's)
    //non-complementary for depth control, complimentary outputs for pitch
    driveESCrev(1, (depth_co * depth_sign) + (pitch_co * pitch_sign));
    driveESCrev(2, (depth_co * depth_sign) + (-pitch_co * pitch_sign));
    
     //Update the output thruster actuators (reversible ESC's)
     //non-complementary for thrust, complimentary outputs for heading
     driveESCrev(3, (thrust * thrust_sign) + (-yaw_co * yaw_sign));
     driveESCrev(4, (thrust * thrust_sign) + (yaw_co * yaw_sign));
}
    
void heartbeat(void){
    static int flag;        
    static int cntr;
    
    switch(flag){        
        case 0:        
            led_pulse = led_pulse + .2;
            if(led_pulse >= 1.0)
                flag = 1;
            break;
        case 1:    
            led_pulse = led_pulse - .2;
            if(led_pulse <= 0.0)
                flag = 2;            
            break;
        case 2:        
            led_pulse = led_pulse + .2;
            if(led_pulse >=1.0)
                flag = 3;
            break;
        case 3:    
            led_pulse = led_pulse - .2;
            if(led_pulse <= .0)
                flag = 4;
            break;
        case 4:
            cntr++;
            if(cntr>40){
                cntr=0;
                flag = 0;    
            }
            break;
        default:
            cntr=0;
            flag=0;
            break;
    }//switch   
    
    //check to see if an ESC character was received
    char c = pc.rxGetLastChar() & 0xFF;        
    if(c == 27){      // Pressed ESC to reset
        NVIC_SystemReset();     //Restart the System
    } 
}

//scales the input control driveVal -1.0 to 1.0 to the Electronic Speed Controllers
// tested parameters (reversible)
void driveESCrev(int esc, float driveVal){
    
    switch(esc){
        case 1:
            float esc1min=1100;
            float esc1max=1900;
            float esc1off = (esc1max + esc1min) / 2.0;
            float esc1scale = (esc1max - esc1min) / 2.0;
                        
            m1_servoOut = (int)((driveVal*escsign_m1)*(esc1scale/2.0))+esc1off;
            break;
        case 2:
            float esc2min=1100;
            float esc2max=1900;
            float esc2scale = (esc2max - esc2min) / 2.0;
            float esc2off = (esc2max + esc2min) / 2.0;
            
            m2_servoOut  = (int)((driveVal*escsign_m2)*(esc2scale/2.0))+esc2off;
            break;
        case 3:
            float esc3min=1100;
            float esc3max=1900;
            float esc3off = (esc3max + esc3min) / 2.0;
            float esc3scale = (esc3max - esc3min) / 2.0;
            
            m3_servoOut  = (int)((driveVal*escsign_m3)*(esc3scale/2.0))+esc3off;
            break;
        case 4:
            float esc4min=1100;
            float esc4max=1900;
            float esc4off = (esc4max + esc4min) / 2.0;
            float esc4scale = (esc4max - esc4min) / 2.0;
            
            m4_servoOut  = (int)((driveVal*escsign_m4)*(esc4scale/2.0))+esc4off;
            break;
    }
}                        

void init_pitchController(void){
    //resets the controllers internals
    pid_pitch.reset();
    //input limits for pitch controller
    pid_pitch.setInputLimits(PITCH_MIN_SP, PITCH_MAX_SP);   //+/- 1 radian
    //Servo Output -1.0 to 1.0
    pid_pitch.setOutputLimits(-0.85, 0.85); //+/- .72
    //If there's a bias.
    pid_pitch.setBias(0.0);
    pid_pitch.setMode(AUTO_MODE);
    
    pid_pitch.setInterval(pitch_Tdelay);
      
    //We want the process variable to be 0.0 at start
    pid_pitch.setSetPoint(pitch_sp);
    pid_pitch.setTunings(pitch_Pk, pitch_Ik, pitch_Dk);
    
    tickerPitch.attach(&pitchController, pitch_Tdelay);    //run pitch controller every 20 milliseconds
    tickerPitchRunning=1;
}

void init_yawController(void){
    //resets the controllers internals
    pid_yaw.reset();
    //input limits for pitch controller
    pid_yaw.setInputLimits(YAW_MIN_SP, YAW_MAX_SP);   //+/- 1 radian
    //Servo Output -1.0 to 1.0
    pid_yaw.setOutputLimits(-0.85, 0.85); //+/- .72
    //If there's a bias.
    pid_yaw.setBias(0.0);
    pid_yaw.setMode(AUTO_MODE);
    
    pid_yaw.setInterval(yaw_Tdelay);
      
    //We want the process variable to be 0.0 at start
    pid_yaw.setSetPoint(yaw_sp);
    pid_yaw.setTunings(yaw_Pk, yaw_Ik, yaw_Dk);
    
    
    tickerYaw.attach(&yawController, yaw_Tdelay);    //run pitch controller every 20 milliseconds
    tickerYawRunning=1;
}

void init_depthController(void){
    //resets the controllers internals
    pid_depth.reset();
    //input limits for pitch controller
    pid_depth.setInputLimits(DEPTH_MIN_SP, DEPTH_MAX_SP);   //0 to 20 meters of depth
    //Servo Output -1.0 to 1.0
    pid_depth.setOutputLimits(-0.85, 0.85); //+/- .72
    //If there's a bias.
    pid_depth.setBias(0.0);
    pid_depth.setMode(AUTO_MODE);
    
    pid_depth.setInterval(depth_Tdelay);
      
    //We want the process variable to be 0.0 at start
    pid_depth.setSetPoint(depth_sp);
    pid_depth.setTunings(depth_Pk, depth_Ik, depth_Dk);
        
    tickerDepth.attach(&depthController, depth_Tdelay);    //run pitch controller every 100 milliseconds
    tickerDepthRunning=1;
}

void depth_calc_offset(void){
    pc.printf("Calculating depth offset\r\n");
    
    float depth_offset_temp = 0.0;
    
    //take average of 50 samples for offset
    for(int i=0;i<50;i++){
        wait(.05);
        float depthtemp = depth_m;                  //get the depth from the periodic sensor readings
        pc.printf("depth = %.3f\r\n", depthtemp);
        depth_offset_temp += depthtemp;        //simply calculates and returns depth, no I2C comms here                                                
    }
    depth_offset_temp /= 50.0;
    depth_offset = depth_offset_temp;   
    pc.printf("depth offset average = %.3f\r\n", depth_offset);
    
}

//------ ASCII command set ----------------------------
void display_commands(void){
    pc.printf("\r\n Command set\r\n");
    pc.printf("? or help - This menu\r\n");    
    wait(.02);
    pc.printf("bnomode - changes orientation mode of IMU\r\n");    
    pc.printf("bnoreadcal - read he calibration data from bno IMU\r\n");    
    pc.printf("bnowritecal - write the calibration data\r\n");
    wait(.02);
    pc.printf("servoout - Control individual servo pulse outputs for testing\r\n");
    wait(.02);
    pc.printf("pitchon - turns on the pitch controller\r\n");
    pc.printf("pitchoff - turns off the pitch controller\r\n");
    pc.printf("pitchp - set the Proportional gain for the pitch controller\r\n");
    wait(.01);
    pc.printf("pitchi - set the Integral gain for the pitch controller\r\n");
    pc.printf("pitchd - set the Derivative gain for the pitch controller\r\n");
    pc.printf("pitchsp - set the pitch setpoint in radians\r\n");
    pc.printf("pitchsign - set the sign of the pitch output\r\n");
    wait(.02);
    pc.printf("yawon - turns on the yaw controller\r\n");
    pc.printf("yawoff - turns off the yaw controller\r\n");
    pc.printf("yawp - set the Proportional gain for the yaw controller\r\n");
    wait(.01);
    pc.printf("yawi - set the Integral gain for the yaw controller\r\n");
    pc.printf("yawd - set the Derivative gain for the yaw controller\r\n");
    pc.printf("yawsp - set the yaw setpoint in radians\r\n");
    pc.printf("yawsign - set the sign of the yaw output\r\n");
    wait(.02);
    pc.printf("depthon - turns on the depth controller\r\n");
    pc.printf("depthoff - turns off the depth controller\r\n");    
    pc.printf("depthp - set the Proportional gain for the depth controller\r\n");
    wait(.01);
    pc.printf("depthi - set the Integral gain for the depth controller\r\n");
    pc.printf("depthd - set the Derivative gain for the depth controller\r\n");
    pc.printf("depthsp - set the depth setpoint in meters\r\n");
    pc.printf("depthsign - set the sign of the depth output\r\n");
    pc.printf("depthzero - calculate and include zero depth offset\r\n");
    wait(.02);
    pc.printf("thrustsp - set the thrust setpoint\r\n");
    pc.printf("thrustsign - set the sign of the thrust output\r\n");
    wait(.02);
    pc.printf("esc - output servo pulse for reversible esc scaled from 0.0-1.0\r\n");
    pc.printf("escsign - change the sign of an individual ESC\r\n");
    pc.printf("SPACEBAR - SPACEBAR turns off all servo pulse outputs (stops thrusters)\r\n");
    wait(.02);
    pc.printf("startcon - start the main actuator controller output\r\n");
    pc.printf("stopcon - stop the main actuator controller output\r\n");
    pc.printf("logon - start logging data to the uSD card\r\n");
    pc.printf("logoff - stop logging data to the uSD card\r\n");
    pc.printf("logtime - change the data logging time per sample in milliseconds (currently %.3f)\r\n", log_sampTime);
    
    wait(.02);
    pc.printf(" CHARACTER CONTROLS\r\n");
    pc.printf("'1' = Yaw Left\r\n");
    pc.printf("'2' = Yaw Right\r\n");
    pc.printf("'3' = Thrust Decrease\r\n");
    pc.printf("'4' = Thrust Increase\r\n");
    pc.printf("'5' = Pitch Decrease\r\n");
    pc.printf("'6' = Pitch Increase\r\n");
    pc.printf("'7' = Depth Increase\r\n");
    pc.printf("'8' = Depth Decrease\r\n");
    pc.printf("' ' = Turn Off ALL Controllers and outputs\r\n");
    pc.printf("'ESCAPE KEY' = Reset Program\r\n");
    
    wait(.02);
    pc.printf("streamon - Turn on serial streaming\r\n");
    pc.printf("streamoff - turn off serial streaming\r\n");
    pc.printf("sermode - select the serial streaming output format\r\n");
        
    wait(.02);    
    
    //wait to receive character before exiting
    while(!pc.readable()){                                 
        char c = pc.getc();
            return;
    }
}

//Initialize the Inertial Measurement Unit (BNO-055)
void bno_init(void){
    if(bno.check()){
        //pc.printf("BNO055 connected\r\n");
        bno.reset();
        wait(.2);
        bno.setmode(OPERATION_MODE_CONFIG);
        bno.SetExternalCrystal(1);
        bno.set_orientation(0);
        
        // page 20-21 BNO055 Datasheet
        //bno.setmode(OPERATION_MODE_IMUPLUS);    //No Magnetometer and relative
        bno.setmode(OPERATION_MODE_NDOF);  //Uses magnetometer with absolute heading
        //bno.setmode(OPERATION_MODE_NDOF_FMC_OFF);   //Uses Mag but relative heading
        bno.set_angle_units(RADIANS);
        wait(.2);
        
        //write calibration data
        for(int i=0;i<22;i++){
            bno.calibration[i] = calib_local[i];
        }
        bno.write_calibration_data();
        
        
    }
    else{
        pc.printf("BNO055 NOT connected\r\n Program Trap.");
        while(1);
    }    
}

void killAllControllers(void){
    tickerUpdateActuators.detach();
    
    tickerPitch.detach();    //Turn off the pitch Ticker
    pitch_co = 0.0;
    
    tickerYaw.detach();    //Turn off the Yaw Ticker
    yaw_co = 0.0;
    
    tickerDepth.detach();    //Turn off the depth Ticker
    depth_co = 0.0;    
}

void readSensors(void){
    bno.get_angles();
    yaw_cor = bno.euler.yaw * YAW640_RATIO; //correct the yaw for radian measurement
    if(yaw_cor > PI)
        yaw_cor = -(PI - yaw_cor) - PI;
    bno.get_calib();
    //bno.get_accel();
    
    read_depthSensor();      //read ADC from last requested depth parameter (requires multiple calls)
    depth_m = depthsensor.depth() -  depth_offset;      //simply calculates and returns depth, no I2C comms here
}
// ---------------- MAIN ------------------------------ 
int main() {
    char cmd_str[30];
    char strbuf[512];       //for data logging
    
    wait(.5);       //allow time for voltages to settle after micro awakens
    pc.baud(115200);
    
    //pc.printf("%s\r\n", __FILE__);  //serial transmit of file name
    wait(.5);
    bno_init();     //Initialize the BNO055 IMU

    tickerPulse.attach(&heartbeat, .05);    //run heartbeat ticker at 50 milliseconds
    tickerReadSensors.attach(&readSensors, .05);    //read the sensors
    //Initialize the depth sensor
    depthsensor.MS5837Reset();
    depthsensor.MS5837Init();    
    //depth_calc_offset();
    
    //Initialize the controllers
    //init_pitchController();
    //init_yawController();
    //init_depthController();
    
    //Start the actuators updates
    //tickerUpdateActuators.attach(&updateActuators, updateActuators_Tdelay);                                                        
   
    while(1) {
        if(stream_flag == 1){
            if(serOutputMode == 0){                           
                //conver data to python list
                pc.printf("["); //Start the list for JSON message
                pc.printf("{""TIME"":""%02d:%02d:%02d:%02d:%03d""}", 
                                runTime.day, runTime.hour, runTime.min, runTime.sec, runTime.ms);               //timestamp
                pc.printf(",");
                pc.printf("{""ROLL"":""%.2f""},{""PITCH"":""%.2f""},{""YAW"":""%.2f""},{""DEPTH"":""%.2f""}", 
                                bno.euler.roll, bno.euler.pitch, yaw_cor, depth_m);                             //IMU, pressure sensor
                pc.printf(",");
                pc.printf("{""PITCHSP"":""%.2f""},{""YAWSP"":""%.2f""},{""DEPTHSP"":""%.2f""}", 
                                pitch_sp, yaw_sp, depth_sp);                                                    //setpoints
                pc.printf(",");
                pc.printf("{""PITCHCO"":""%.2f""},{""YAWCO"":""%.2f""},{""DEPTHCO"":""%.2f""}", 
                                pitch_co, yaw_co, depth_co);                                                     //controller outputs
                pc.printf(",");
                pc.printf("{""THRUST"":""%.2f""}", 
                                thrust);                                                                        //Thrust
                pc.printf(",");
                pc.printf("{""S1"":""%d""},{""S2"":""%d""},{""S3"":""%d""},{""S4"":""%d""}", 
                                m1_servoOut.read(), m2_servoOut.read(), m3_servoOut.read(), m4_servoOut.read());//servo pulses
                pc.printf("]"); //Stop the list for JSON message                        
                
                pc.printf("\r\n");                                                                                //string terminator
            }
            //This output mode is good for data logging (Excel, MATLAB, etc)
            if(serOutputMode == 1){     
                //Serial output (for logging, etc)
                pc.printf("T:%02d:%02d:%02d:%02d:%03d,", runTime.day, runTime.hour, runTime.min, runTime.sec, runTime.ms);                
                pc.printf("ROLL:%.2f,PITCH:%.2f,YAW:%.2f,DEPTH:%.2f,", bno.euler.roll, bno.euler.pitch, yaw_cor, depth_m);
                pc.printf("PITCHSP:%.2f,YAWSP:%.2f,DEPTHSP:%.2f,", pitch_sp, yaw_sp, depth_sp);
                pc.printf("PITCHCO:%.3f,YAWCO:%.3f,DEPTHCO:%.3f,", pitch_co, yaw_co, depth_co);               
                pc.printf("THRUST:%.3f,", thrust);               
                pc.printf("S1:%d,S2:%d,S3:%d,S4:%d", m1_servoOut.read(), m2_servoOut.read(), m3_servoOut.read(), m4_servoOut.read());           
                pc.printf("\r\n");
            }
            //This output mode is good for visually friendly terminal stuff
            if(serOutputMode >= 2){
                pc.printf("%s", CLR_SCR);     
                //Serial output (for logging, etc)
                pc.printf("%s", RED_FONT);
                pc.printf("%T:%02d:%02d:%02d:%02d:%03d\r\n", runTime.day, runTime.hour, runTime.min, runTime.sec, runTime.ms);                
                pc.printf("%s", GREEN_FONT);
                pc.printf("ROLL:%5.2f PITCH:%5.2f YAW:%5.2f DEPTH:%5.2f\r\n", bno.euler.roll, bno.euler.pitch, yaw_cor, depth_m);
                pc.printf("%s", YELLOW_FONT);
                pc.printf("PITCHSP:%5.2f YAWSP:%5.2f DEPTHSP:%5.2f\r\n", pitch_sp, yaw_sp, depth_sp);
                pc.printf("%s", BLUE_FONT);
                pc.printf("PITCHCO:%5.3f YAWCO:%5.3f DEPTHCO:%5.3f\r\n", pitch_co, yaw_co, depth_co);               
                pc.printf("%s", WHITE_FONT);
                pc.printf("THRUST:%5.3f\r\n", thrust);
                pc.printf("%s", DEFAULT_FONT);               
                pc.printf("S1:%5d S2:%5d S3:%5d S4:%5d\r\n", m1_servoOut.read(), m2_servoOut.read(), m3_servoOut.read(), m4_servoOut.read());           
                //pc.printf("\r\n");
                
                wait(.05);  //additional wait time for visual
            }
         }//stream_flag == 1   
        
        if(log_flag == 1)
        {                        
        //Initialize file for SD card logging
            if(runTime.ms_total > log_nextSamp){
                FILE *sdFile = fopen("/sd/sdfile.txt", "a");
                if(sdFile == NULL){
                    pc.printf("\r\nError - Could not open SD card file for write\r\n");
                    log_flag = 0;
                    wait(1);
                }
                else{
                    led_log = 1;        //indicator for logging
                    //Serial output (for logging, etc)
                    sprintf(strbuf, "T:%02d:%02d:%02d:%02d:%03d,", runTime.day, runTime.hour, runTime.min, runTime.sec, runTime.ms);
                    fprintf(sdFile, "%s", strbuf);                
                    sprintf(strbuf, "ROLL:%.2f,PITCH:%.2f,YAW:%.2f,", bno.euler.roll, bno.euler.pitch, yaw_cor);
                    fprintf(sdFile, "%s", strbuf);
                    sprintf(strbuf, "PITCHSP:%.2f,YAWSP:%.2f,DEPTHSP:%.2f,", pitch_sp, yaw_sp, depth_sp);
                    fprintf(sdFile, "%s", strbuf);
                    sprintf(strbuf, "PITCHCO:%.3f,YAWCO:%.3f,DEPTHCO:%.3f,", pitch_co, yaw_co, depth_m);
                    fprintf(sdFile, "%s", strbuf);                
                    sprintf(strbuf, "THRUST:%.3f,", thrust);
                    fprintf(sdFile, "%s", strbuf);                
                    sprintf(strbuf, "S1:%d,S2:%d,S3:%d,S4:%d\r", m1_servoOut.read(), m2_servoOut.read(), m3_servoOut.read(), m4_servoOut.read());
                    fprintf(sdFile, "%s", strbuf);
                    
                    fclose(sdFile);
                    led_log = 0;
                }
                
                log_nextSamp = runTime.ms_total + log_sampTime;
            }//log next samp
        }//log
                
        //pc.printf("Roll=%6.3f Pitch=%6.3f  Yaw=%6.3f ", -bno.euler.roll, bno.euler.pitch, bno.euler.yaw);
        //pc.printf("STAT= S%1d G%1d A%1d M%1d ", sys_stat,gyr_stat,acc_stat,mag_stat);
        //pc.printf("Accel X = %.3f Accel Y = %.3f Accel Z = %.3f ", bno.accel.x, bno.accel.y, bno.accel.z);
        
        //pc.printf("sp=%.3f cor=%.3f yawT_error=%.3f, co=%.3f yaw_error=%.3f yaw_sign=%.1f %d errU=%.3f", 
        //            yaw_sp, yaw_cor, yaw_temp_error, yaw_co, yaw_error, yaw_sign, con_state, yaw_err_unwrapped);
        
        wait(.02);
        
        if(pc.readable()){            //if data in receive buffer
            //SINGLE CHARACTER RETURNS FOR ARROWS AND SPACEBAR
            char c = pc.rxGetLastChar();
            //pc.printf("\r\nchar = 0x%4X\r\n", c);
            //wait(.1);
            if(c == 27){      // Pressed ESC to reset
                NVIC_SystemReset();     //Restart the System
            }
            /*
            else if(c == 0xE0){      // MAKE and BREAK codes for arrows
                c = pc.getc();     //pull last char from buffer
                c = pc.rxGetLastChar(); //then read the next one
                pc.printf("\r\nchar = 0x%4X\r\n", c);
                if(c == 0x72)                       //down arrow
                    thrust -= .05;
                if(c == 0x75)                       //up arrow
                    thrust += .05;
                if(c == 0x74)                       //right arrow
                    yaw_sp += .05;
                if(c == 0x6B)                       //left arrow
                    yaw_sp -= .05;

                //clear out the rest of the buffer                    
               pc.rxBufferFlush();
            }
            */
            else if(c == '1'){      // yaw left
                yaw_sp -= .05;
                //clear out the rest of the buffer                    
               pc.rxBufferFlush();
            }
            else if(c == '2'){      // yaw right
                yaw_sp += .05;
                //clear out the rest of the buffer                    
               pc.rxBufferFlush();
            }
            else if(c == '3'){      // thrust decrease
                thrust -= .05;
                //clear out the rest of the buffer                    
               pc.rxBufferFlush();
            }
            else if(c == '4'){      // thrust increase
                thrust += .05;
                //clear out the rest of the buffer                    
                pc.rxBufferFlush();
            }
            else if(c == '5'){      // pitch decrease
                pitch_sp -= .05;
                //clear out the rest of the buffer                    
               pc.rxBufferFlush();
            }
            else if(c == '6'){      // pitch increase
                pitch_sp += .05;
                //clear out the rest of the buffer                    
                pc.rxBufferFlush();
            }            
            else if(c == '7'){      // Increase depth
                depth_sp += .05;
                //clear out the rest of the buffer                    
                pc.rxBufferFlush();
            }
            else if(c == '8'){      // Depth depth
                depth_sp -= .05;
                //clear out the rest of the buffer                    
                pc.rxBufferFlush();
            }
            else if(c == ' '){                // ' ' (SPACEBAR) stops all servo outputs
                tickerUpdateActuators.detach();
                //turn off all pulsewidths (disable all ESCs?)
                m1_servoOut=0;
                m2_servoOut=0;
                m3_servoOut=0;
                m4_servoOut=0;
                
                tickerUpdateActuators.detach();
                tickerPitch.detach();    //Turn off the pitch Ticker
                pitch_co = 0.0;
                tickerYaw.detach();    //Turn off the Yaw Ticker
                yaw_co = 0.0;
                tickerDepth.detach();    //Turn off the depth Ticker
                depth_co = 0.0;
                pc.rxBufferFlush();
            }
            
            // IF NOT ONE OF THE SINGLE CHARACTER COMMANDS, START STRING COMNMANDS
            else{
                led_sort = 1;                
                pc.scanf("%s", cmd_str);//read the string
                
                //display the command set
                if(!strcmp(cmd_str, "?") || !strcmp(cmd_str, "help")){
                    display_commands();
                }
                //change bno orientation mode
                if(!strcmp(cmd_str, "bnomode")){
                    int mode=0;
                    pc.printf("\r\nbno orientation\r\nEnterMode:");
                    pc.scanf("%d", &mode);//read the string
                    pc.printf("\r\n%d orientation mode entered", mode);
                    
                    wait(.1);
                    bno.reset();
                    wait(.2);
                    bno.setmode(OPERATION_MODE_CONFIG);
                    bno.SetExternalCrystal(1);
                    bno.set_orientation(mode);
                    bno.setmode(OPERATION_MODE_NDOF);  //Uses magnetometer
                    //bno.setmode(OPERATION_MODE_NDOF_FMC_OFF);   //no magnetometer
                    bno.set_angle_units(RADIANS);
                    wait(.02);           
                }
                // read the bno calibration values
                if(!strcmp(cmd_str, "bnoreadcal")){
                    bno.read_calibration_data();
                    pc.printf("Calibration data read is as follows...\r\n");
                    for(int i=0;i<22;i++){
                        pc.printf("reg=0x%2X byte[%d]=0x%02X\r\n", i+55, i, bno.calibration[i]);
                        wait(.02);
                    }
                    
                    pc.printf("\r\n\r\nCopy the C-Array below.\r\n");
                    pc.printf("\r\nchar calib_local[] = {");
                    for(int i=0;i<22;i+=2){
                        pc.printf("0x%02X,0x%02X", bno.calibration[i], bno.calibration[i+1]);                                        
                        if(i==20)
                            pc.printf("};\r\n");
                        else
                            pc.printf(",\r\n\t\t\t");
                        wait(.02);
                    }
                    
                    pc.printf("\r\n\r\nPress key to continue.\r\n");
                    while(!pc.readable());
                    char c = pc.getc();
                    c=0;
                }
                
               //write calibration data
                if(!strcmp(cmd_str, "bnowritecal")){
                    for(int i=0;i<22;i++){
                        bno.calibration[i] = calib_local[i];
                    }
                    bno.write_calibration_data();
                    pc.printf("Calibration data written is as follows...\r\n");
                    for(int i=0;i<22;i++){
                        pc.printf("reg=0x%2X byte[%d]=0x%02X\r\n", i+55, i, bno.calibration[i]);
                        wait(.02);
                    }
                    pc.printf("\r\n\r\nPress key to continue.\r\n");
                    while(!pc.readable());
                    char c = pc.getc();
                    c=0;                
                }
                //-------------- PITCH ----------------------------------------------------
                //Turn ON the Pitch controller
                if(!strcmp(cmd_str, "pitchon")){
                    
                    init_pitchController();
                    pc.printf("Pitch Controller is ON\r\n");
                }
    
                //Turn OFF the Pitch controller
                if(!strcmp(cmd_str, "pitchoff")){
                    
                    tickerPitch.detach();    //Turn off the pitch Ticker
                    pitch_co = 0.0;
                    pc.printf("Pitch Controller is OFF\r\n");
                }
                
                //change the pitch proportional gain
                if(!strcmp(cmd_str, "pitchp")){
                    float Pk_temp;
                    pc.printf("Set pitch proportional gain (currently %.3f)\r\n", pitch_Pk);
                    pc.scanf("%f", &Pk_temp);
                    pitch_Pk = Pk_temp;
                    
                    pid_pitch.setTunings(pitch_Pk, pitch_Ik, pitch_Dk);
                    
//                    pc.printf("Pitch Proportional gain is %.3f\r\n", pitch_Pk);
//                    pc.printf("\r\n\r\nPress key to continue.\r\n");
//                    while(!pc.readable());
//                    char c = pc.getc();
//                    c=0;
                }
                
                //change the integral gain for the pitch controller
                if(!strcmp(cmd_str, "pitchi")){
                    float Ik_temp;
                    pc.printf("Set pitch integral gain (currently %.3f)\r\n", pitch_Ik);
                    pc.scanf("%f", &Ik_temp);
                    pitch_Ik = Ik_temp;
                    
                    pid_pitch.setTunings(pitch_Pk, pitch_Ik, pitch_Dk);
                }            
                
                //change the derivative gain for the pitch controller
                if(!strcmp(cmd_str, "pitchd")){
                    float Dk_temp;
                    pc.printf("Set pitch derivative gain (currently %.7f)\r\n", pitch_Dk);
                    pc.scanf("%f", &Dk_temp);
                    pitch_Dk = Dk_temp;
                    
                    pid_pitch.setTunings(pitch_Pk, pitch_Ik, pitch_Dk);
                }            
                
                //change the set point on the pitch controller
                if(!strcmp(cmd_str, "pitchsp")){
                    float temp_sp;
                    pc.printf("Set pitch (currently %.3f)\r\n", pitch_sp);
                    pc.scanf("%f", &temp_sp);
                    
                    if(temp_sp < PITCH_MIN_SP){
                        temp_sp = PITCH_MIN_SP;
                        pc.printf("Pitch Entered Exceeds min value of %.3f\r\n", PITCH_MIN_SP);
                    }
                    if(temp_sp > PITCH_MAX_SP){
                        temp_sp = PITCH_MAX_SP;
                        pc.printf("Pitch Entered Exceeds max value of %.3f\r\n", PITCH_MAX_SP);
                    }
                    pitch_sp = temp_sp;
                    pc.printf("Pitch is %.3f\r\n", pitch_sp);
                }
                
                //change the sign of the pitch controller
                if(!strcmp(cmd_str, "pitchsign")){
                    pc.printf("Set pitch (currently %.1f)\r\n", pitch_sign);
                    pc.scanf("%f", &pitch_sign);
                    
                    if(pitch_sign < 0.0){
                        pitch_sign = -1.0;                    
                    }
                    if(pitch_sign >= 0.0){
                        pitch_sign = 1.0;                    
                    }
                    
                    pc.printf("Pitch sign is %.3f\r\n", pitch_sign);
                }            
                
                //---------------- YAW ---------------------------------
                //Turn ON the Yaw controller
                if(!strcmp(cmd_str, "yawon")){
                    
                    init_yawController();
                    pc.printf("Yaw Controller is ON\r\n");
                }
    
                //Turn OFF the Yaw controller
                if(!strcmp(cmd_str, "yawoff")){
                    
                    tickerYaw.detach();    //Turn off the Yaw Ticker
                    yaw_co = 0.0;
                    pc.printf("Yaw Controller is OFF\r\n");
                }
                
                //change the yaw proportional gain
                if(!strcmp(cmd_str, "yawp")){
                    float Pk_temp;
                    pc.printf("Set yaw proportional gain (currently %.3f)\r\n", yaw_Pk);
                    pc.scanf("%f", &Pk_temp);
                    yaw_Pk = Pk_temp;
                    
                    pid_yaw.setTunings(yaw_Pk, yaw_Ik, yaw_Dk);
                }
                
                //change the integral gain for the yaw controller
                if(!strcmp(cmd_str, "yawi")){
                    float Ik_temp;
                    pc.printf("Set yaw integral gain (currently %.3f)\r\n", yaw_Ik);
                    pc.scanf("%f", &Ik_temp);
                    yaw_Ik = Ik_temp;
                    
                    pid_yaw.setTunings(yaw_Pk, yaw_Ik, yaw_Dk);
                }            
                
                //change the derivative gain for the yaw controller
                if(!strcmp(cmd_str, "yawd")){
                    float Dk_temp;
                    pc.printf("Set yaw derivative gain (currently %.7f)\r\n", yaw_Dk);
                    pc.scanf("%f", &Dk_temp);
                    yaw_Dk = Dk_temp;
                    
                    pid_yaw.setTunings(yaw_Pk, yaw_Ik, yaw_Dk);
                    
                    pc.printf("yaw derivative gain is %.7f\r\n", yaw_Dk);
                }            
                
                //change the set point on the yaw controller
                if(!strcmp(cmd_str, "yawsp")){
                    float temp_sp;
                    pc.printf("Set yaw (currently %.3f)\r\n", yaw_sp);
                    pc.scanf("%f", &temp_sp);
                    
                    if(temp_sp < YAW_MIN_SP){
                        temp_sp = YAW_MIN_SP;
                        pc.printf("yaw Entered Exceeds min value of %.3f\r\n", YAW_MIN_SP);
                    }
                    if(temp_sp > YAW_MAX_SP){
                        temp_sp = YAW_MAX_SP;
                        pc.printf("yaw Entered Exceeds max value of %.3f\r\n", YAW_MAX_SP);
                    }
                    yaw_sp = temp_sp;
                    pc.printf("Yaw is %.3f\r\n", yaw_sp);
                }
    
                //change the sign of the yaw controller
                if(!strcmp(cmd_str, "yawsign")){
                    pc.printf("Set yaw (currently %.1f)\r\n", yaw_sign);
                    pc.scanf("%f", &yaw_sign);
                    
                    if(yaw_sign < 0.0){
                        yaw_sign = -1.0;                    
                    }
                    if(yaw_sign >= 0.0){
                        yaw_sign = 1.0;                    
                    }
                    
                    pc.printf("Yaw sign is %.3f\r\n", yaw_sign);                
                }            
    
                //---------------- DEPTH ---------------------------------
                //Calcuate the zero offset for depth
                if(!strcmp(cmd_str, "depthzero")){
                    depth_calc_offset();                                
                }            
                //Turn ON the Depth controller
                if(!strcmp(cmd_str, "depthon")){
                    
                    init_depthController();
                    pc.printf("Depth Controller is ON\r\n");
                }
    
                //Turn OFF the Depth controller
                if(!strcmp(cmd_str, "depthoff")){
                    
                    tickerDepth.detach();    //Turn off the depth Ticker
                    depth_co = 0.0;
                    pc.printf("Depth Controller is OFF\r\n");
                }
                //change the yaw proportional gain
                if(!strcmp(cmd_str, "depthp")){
                    float Pk_temp;
                    pc.printf("Set depth proportional gain (currently %.3f)\r\n", depth_Pk);
                    pc.scanf("%f", &Pk_temp);
                    depth_Pk = Pk_temp;
                    
                    pid_depth.setTunings(depth_Pk, depth_Ik, depth_Dk);
                    
                    pc.printf("depth Proportional gain is %.3f\r\n", depth_Pk);
                }
                
                //change the integral gain for the depth controller
                if(!strcmp(cmd_str, "depthi")){
                    float Ik_temp;
                    pc.printf("Set depth integral gain (currently %.3f)\r\n", depth_Ik);
                    pc.scanf("%f", &Ik_temp);
                    depth_Ik = Ik_temp;
                    
                    pid_depth.setTunings(depth_Pk, depth_Ik, depth_Dk);
                    
                    pc.printf("depth integral gain is %.3f\r\n", depth_Ik);
                }            
                
                //change the derivative gain for the depth controller
                if(!strcmp(cmd_str, "depthd")){
                    float Dk_temp;
                    pc.printf("Set depth derivative gain (currently %.7f)\r\n", depth_Dk);
                    pc.scanf("%f", &Dk_temp);
                    depth_Dk = Dk_temp;
                    
                    pid_depth.setTunings(depth_Pk, depth_Ik, depth_Dk);
                    
                    pc.printf("depth derivative gain is %.7f\r\n", depth_Dk);
                }            
                
                //change the set point on the depth controller
                if(!strcmp(cmd_str, "depthsp")){
                    float temp_sp;
                    pc.printf("Set depth (currently %.3f)\r\n", depth_sp);
                    pc.scanf("%f", &temp_sp);
                    
                    if(temp_sp < DEPTH_MIN_SP){
                        temp_sp = DEPTH_MIN_SP;
                        pc.printf("depth Entered Exceeds min value of %.3f\r\n", DEPTH_MIN_SP);
                    }
                    if(temp_sp > DEPTH_MAX_SP){
                        temp_sp = DEPTH_MAX_SP;
                        pc.printf("depth Entered Exceeds max value of %.3f\r\n", DEPTH_MAX_SP);
                    }
                    depth_sp = temp_sp;
                    pc.printf("Depth is %.3f\r\n", depth_sp);
                }
                
                //change the sign of the depth controller
                if(!strcmp(cmd_str, "depthsign")){
                    pc.printf("Set depth (currently %.1f)\r\n", depth_sign);
                    pc.scanf("%f", &depth_sign);
                    
                    if(depth_sign < 0.0){
                        depth_sign = -1.0;                    
                    }
                    if(depth_sign >= 0.0){
                        depth_sign = 1.0;                    
                    }
                    
                    pc.printf("Depth sign is %.3f\r\n", depth_sign);                
                }            
    
                //individually test servo pulse outputs
                if(!strcmp(cmd_str, "servoout")){
                    tickerUpdateActuators.detach();
                    //turn off all pulsewidths to 1000 (stops motors faster then 0
                    m1_servoOut=1500;
                    m2_servoOut=1500;
                    m3_servoOut=1500;
                    m4_servoOut=1500;
                    
                    int quitFlag=0;
                    while(!quitFlag){
                        pc.printf("Enter servo output to Drive (1-4) or'q' to quit:");
                        int servoNum=0;
                        pc.scanf("%d", &servoNum);
                        if(servoNum<1 || servoNum>4 || servoNum == 'q'){
                            pc.printf("Invalid Servo number entered!Exiting Now!\r\n");
                            quitFlag=1;
                            break;
                        }
                        else{
                            pc.printf("Servo %d entered\r\n Enter micro-second pulse width value:", servoNum);
                            int pwusVal=0;
                            pc.scanf("%d", &pwusVal);
                            pc.printf("\r\npulse width of %d micro-seconds entered\r\n", pwusVal);
                            
                            switch(servoNum){
                                case 1:
                                    m1_servoOut=pwusVal;
                                    break;
                                case 2:
                                    m2_servoOut=pwusVal;
                                    break;
                                case 3:
                                    m3_servoOut=pwusVal;
                                    break;
                                case 4:
                                    m4_servoOut=pwusVal;
                                    break;
                                default:
                                    quitFlag=1;
                                    break;
                            }
                        }    
                    }
                }//servoout command
                
                //drive the ESC to calibrated servo pulse outputs (reversible ESC)
                if(!strcmp(cmd_str, "esc")){
                    tickerUpdateActuators.detach();
                    //turn off all pulsewidths (disable all ESCs?)
                    m1_servoOut=0;
                    m2_servoOut=0;
                    m3_servoOut=0;
                    m4_servoOut=0;
                    
                    int quitFlag=0;
                    while(!quitFlag){
                        pc.printf("Enter servo output to Drive (1-4) or'q' to quit:");
                        int servoNum=0;                        
                        pc.scanf("%d", &servoNum);
                        
                        if(servoNum<1 || servoNum>4 || servoNum == 'q'){
                            pc.printf("Invalid Servo number entered!Exiting Now!\r\n");
                            quitFlag=1;
                            break;
                        }
                        else{                                                                                                                                               
                            float pwscaledVal;
                            
                            pc.printf("Servo %d entered\r\n Enter pulse output scaled from (-1.0 to 1.0):", servoNum);                        
                            pc.scanf("%f", &pwscaledVal); 
                            
                            //keep in bounds
                            if(pwscaledVal < -1.0)
                                pwscaledVal = -1.0;
                            if(pwscaledVal > 1.0)
                                pwscaledVal = 1.0;
                                
                            pc.printf("\r\npulse width of %.3f entered\r\n", pwscaledVal);
                                                
                            driveESCrev(servoNum, pwscaledVal);
                        }
                    }
                }//driveESCrev                
                
                // start the main combined actuator controller
                if(!strcmp(cmd_str, "startcon")){
                    m1_servoOut=1500;
                    m2_servoOut=1500;
                    m3_servoOut=1500;
                    m4_servoOut=1500;
                    
                    wait(2);
                    tickerUpdateActuators.attach(&updateActuators, updateActuators_Tdelay);
                    pc.printf("OK\r\n");
                }
    
                // stop the main combined actuator controller
                if(!strcmp(cmd_str, "stopcon")){                
                    tickerUpdateActuators.detach();
                }            
                //thrust sign command
                if(!strcmp(cmd_str, "thrustsign")){
                    pc.printf("Set thrust sign (currently %.1f)\r\n", thrust_sign);
                    pc.scanf("%f", &thrust_sign);
                    
                    if(thrust_sign < 0.0){
                        thrust_sign = -1.0;
                    }
                    if(thrust_sign >= 0.0){
                        thrust_sign = 1.0;
                    }
                    
                    pc.printf("Thrust sign is %.3f\r\n", thrust_sign);
                }                        
                
                //thrust command
                if(!strcmp(cmd_str, "thrustsp")){
                    pc.printf("Set thrust (currently %.1f)\r\n", thrust);
                    pc.scanf("%f", &thrust);
                    
                    //Keep thrust within bounds
                    if(thrust < THRUST_MIN_SP){
                        thrust = THRUST_MIN_SP;
                    }
                    if(thrust >= THRUST_MAX_SP){
                        thrust = THRUST_MAX_SP;
                    }
                    
                    pc.printf("Thrust is %.3f\r\n", thrust);
                }
                
                //change sign of single ESC
                if(!strcmp(cmd_str, "escsign")){
                    
                    int quitFlag=0;
                    while(!quitFlag){           
                        pc.printf("Change ESC sign.\r\n Select ESC to change sign (1-4) or 'q' to quit:");
                        int escnum=0;
                        pc.scanf("%d", &escnum);
                        
                        if(escnum < 1 || escnum > 4 || escnum == 'q'){
                            pc.printf("Exiting.\r\n");
                            quitFlag = 1;                            
                        }
                        else{
                            float tempsign;                            
                                                        
                            switch(escnum){
                                case 1:
                                    pc.printf("ESC1 selected (currently %.1f). Select sign:", escsign_m1);
                                    pc.scanf("%f", &tempsign);
                                    pc.printf("sign=%.1f", tempsign);
                                    escsign_m1 = tempsign;                                                                                                            
                                break;
                                
                                case 2:
                                    pc.printf("ESC2 selected (currently %.1f). Select sign:", escsign_m2);
                                    pc.scanf("%f", &tempsign);
                                    pc.printf("sign=%.1f", tempsign);
                                    escsign_m2 = tempsign;
                                break;
                                
                                case 3:
                                    pc.printf("ESC3 selected (currently %.1f). Select sign:", escsign_m3);
                                    pc.scanf("%f", &tempsign);
                                    pc.printf("sign=%.1f", tempsign);                                
                                    escsign_m3 = tempsign;
                                break;
                                
                                case 4:
                                    pc.printf("ESC4 selected (currently %.1f). Select sign:", escsign_m4);
                                    pc.scanf("%f", &tempsign);
                                    pc.printf("sign=%.1f", tempsign);                                
                                    escsign_m4 = tempsign;
                                break;
                                
                                default:
                                    quitFlag=1;     //set the flag to quit
                                    break;    
                            }
                        }
                    }
                }

                // Turn on the uSD data logging
                if(!strcmp(cmd_str, "logon")){                
                    log_flag = 1;
                    log_nextSamp = runTime.ms_total + log_sampTime;
                }
                // Turn off the uSD data logging
                if(!strcmp(cmd_str, "logoff")){
                    log_flag = 0;
                }
                // Change the uSD data logging time per sample
                if(!strcmp(cmd_str, "logtime")){                
                    pc.printf("Enter log time per sample (in milliseconds):");
                    pc.scanf("%f", &log_sampTime);
                    
                    log_nextSamp = runTime.ms_total + log_sampTime;
                }
                // Turn on the serial streaming
                if(!strcmp(cmd_str, "streamon")){
                    stream_flag = 1;
                }
                // Turn off the serial streaming
                if(!strcmp(cmd_str, "streamoff")){
                    stream_flag = 0;
                }
                // change the serial output mode
                if(!strcmp(cmd_str, "sermode")){
                    pc.printf("Enter Serial output mode:\r\n");
                    pc.printf(" 0 - JSON meggage format\r\n");
                    pc.printf(" 1 - CSV format\r\n");
                    pc.printf(" 2 - termnal user friendly\r\n");
                    
                    pc.scanf("%d", &serOutputMode);
                    pc.printf("Serial Output Mode %d Entered\r\n", serOutputMode);
                    log_flag = 0;
                }                                 
            }//else STRING COMMANDS
        }//if pc.readable()
        led_sort = 0;
/*
        //Update the actuator outputs (reversible ESC's)
        //non-complementary for depth control, complimentary outputs for pitch
        driveESCrev(1, (depth_co * depth_sign) + (pitch_co * pitch_sign));
        driveESCrev(2, (depth_co * depth_sign) + (-pitch_co * pitch_sign));            
        
         //Update the output thruster actuators (reversible ESC's)
         //non-complementary for thrust, complimentary outputs for heading
         driveESCrev(3, (thrust * thrust_sign) + (-yaw_co * yaw_sign));
         driveESCrev(4, (thrust * thrust_sign) + (yaw_co * yaw_sign));
*/             
    }//while(1)
}//main()
