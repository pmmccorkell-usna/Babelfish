// AUV test program for reading IMU and depth data and controlling vehicle state
// J. Bradshaw 20190723


#include "mbed.h"
#include "BNO055.h"
#include "PID.h"
#include "RunTimer.h"
#include "ServoOut.h"
#include "MS5837.h"

#define PI 3.1415926545359f

#define PITCH_MIN_SP    (-1.5f)         // Min Pitch in radians
#define PITCH_MAX_SP    (1.5)           // Max Pitch in radians

#define YAW_MIN_SP      (0.0f)          // Min Yaw in radians
#define YAW_MAX_SP      (2.0 * PI)      // Max Yaw in radians
#define YAW640_RATIO    (.981746f)      //for some reason, the YAW RADIAN measurement from the BNO
                                        // yeilds 0.0 - 6.40 instead of 2*PI, this scales it accordingly
#define DEPTH_MIN_SP    (0.0f)          // Min Depth in meters
#define DEPTH_MAX_SP    (3.0)           // Max Depth in meters

#define THRUST_MIN_SP   (-0.6)           //Min Thrust value (currently ESC normalized val from -1.0 to 1.0)
#define THRUST_MAX_SP   (0.6)            //Max Thrust value (currently ESC normalized val from -1.0 to 1.0)

//Instantiate classes
Serial pc(USBTX, USBRX);
BNO055 bno(p9, p10);   // (SDA, SCL) or... SDA is p28, SCL is p27
MS5837 depthsensor(p9, p10, ms5837_addr_no_CS);
PID pid_pitch(0.0, 0.0, 0.0, .02);
//PID pid_yaw(0.0, 0.0, 0.0, .02);      //this is done manually due to wrap around sign change
PID pid_depth(0.0, 0.0, 0.0, .1);

Ticker tickerPulse;

//Run Timer for total runtime (regular timer class wraps too early)
RunTimer runTime;

PwmOut led_pulse(LED1);
ServoOut m1_servoOut(p21);     //Top Fore
ServoOut m2_servoOut(p22);     //Top Aft
ServoOut m3_servoOut(p23);     //starboard thruster
ServoOut m4_servoOut(p24);     //port thruster

//pitch controller global variables
Ticker tickerPitch;
volatile float pitch_Tdelay = .02;
volatile float pitch_Pk =.220f;
volatile float pitch_Ik =0.0;   //0.025f;
volatile float pitch_Dk =0.00001;
volatile float pitch_sp = 0.0;      //initialize at 0.0 rad position pitch
volatile float pitch_co = 0.0;
volatile float pitch_sign = 1.0;    // should be +1.0 or -1.0

//yaw/heading controller global variables
Ticker tickerYaw;
volatile float yaw_Tdelay = .02;
volatile float yaw_Pk =1.0f;
volatile float yaw_Ik =.50f;      //.055
volatile float yaw_Dk =0.0;
volatile float yaw_sp = 0.0;      //initialize at 0.0 rad position pitch
volatile float yaw_co = 0.0;
volatile float yaw_sign_calc=0.0;           //used during direction calculation difference
volatile float yaw_sign=1.0;                //user programmable sign applied to output actuators
float yaw_error=0.0;     //make global for now for testing purposes
float yaw_err, yaw_P, yaw_I, yaw_D;

float yaw_err_last=0.0;     //make global for now for testing purposes
float yaw_temp=0.0;     //make global for now for testing purposes
float yaw_cor=0.0;      //corrected yaw heading in radians
float yaw_I_saturation = .4;        //saturation value for integral component

//depth controller global variables
Ticker tickerDepth;
volatile float depth_Tdelay = .1;       //100 millisecond update
volatile float depth_Pk =.60f;
volatile float depth_Ik =.50f;      //.055
volatile float depth_Dk =0.0;
volatile float depth_sp = 0.0;      //initialize at 0.0 meters of depth
volatile float depth_co = 0.0;
volatile float depth_sign = 1.0;    // should be +1.0 or -1.0

// Eventually make a controller for the thrust, but for now
//  just use this variable to add to he thruster ESC's (-1.0 to 1.0)
float thrust = 0.0;                 // -1.0 to 1.0 output for reversible ESC's
float thrust_sign = 1.0;            // Allows the user can change the sign of the 
                                    // thrust output manually

//main function that updates the controller outputs and drives actuators
Ticker ticketUpdateActuators;

char calib_local[] = {0x00,0x00,
                0x00,0x00,
                0x00,0x00,
                0x28,0x00,
                0x42,0x00,
                0x15,0x01,
                0x00,0x00,
                0x00,0x00,
                0x00,0x00,
                0xE8,0x03,
                0x84,0x02};                        

float testvalf;         //Universal test floating point value

//----- Prototypes ------------------------
void pitchController(void);
void yawController(void);
void depthController(void);
void heartbeat(void);
void driveESC(int esc, float driveVal);
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

void yawController(void){
    //first check for the sign, which direction is faster to turn in
    // This is why the PID is done manually as the sign change from the
    // 2 * PI wrap-around
    if(yaw_sp >= yaw_cor)
    {
        yaw_error = yaw_sp - yaw_cor;
        if(yaw_error <= PI)        //179
            yaw_sign_calc=1.0;
        if(yaw_error > PI)        //179
            yaw_sign_calc=-1.0;
    }
    
    if(yaw_sp < yaw_cor)
    {
        yaw_error = yaw_cor - yaw_sp;
        if(yaw_error <= PI)    //179
            yaw_sign_calc=-1.0;
        if(yaw_error > PI)    //180
            yaw_sign_calc=1.0;
    }

    if((yaw_error >= PI) && (yaw_cor != yaw_sp))
        yaw_temp = (2.0*PI) - yaw_error;        //360
    else
        yaw_temp = yaw_error;
            
     yaw_err = (yaw_temp/PI) * yaw_sign_calc;     //normalize and sign
     yaw_P = yaw_Pk*yaw_err;      //P = gain times error
     
     //put a saturation boundary on the integral
     yaw_I += (yaw_Ik*yaw_err)*yaw_Tdelay;
     if(yaw_I > yaw_I_saturation)
        yaw_I = yaw_I_saturation;
      if(yaw_I < -yaw_I_saturation)
        yaw_I = -yaw_I_saturation;
    
     yaw_D = ((yaw_err - yaw_err_last)* yaw_Tdelay)* yaw_Dk;
     
     //add the PID components together
     yaw_co = yaw_P + yaw_I + yaw_Dk;
     
     yaw_err_last = yaw_temp;           //update history     
     
  //   driveESCrev(2, yaw_co);
     
     /*
    //set the set point
    pid_yaw.setSetPoint(yaw_temp*yaw_sign);
    
    //Update the process variable.
    pid_yaw.setProcessValue(yaw_cor);
            
    //Set the new output.
    yaw_co = pid_yaw.compute();
    */    
}

void depthController(void){
    //read the depth sensor
    depthsensor.Barometer_MS5837();

    float depth_m = depthsensor.depth();
    
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
}

//scales the input control driveVal 0.0-1.0 to the Electronic Speed Controllers
// tested parameters (non-reversible)
void driveESC(int esc, float driveVal){
    
    switch(esc){
        case 1:
            float esc1min=1050;
            float esc1max=1900;
            float esc1off = (esc1max + esc1min) / 2.0;
            float esc1scale = (esc1max - esc1min) / 2.0;
            
            m1_servoOut = (int)(driveVal*esc1scale)+esc1off;
            break;
        case 2:
            float esc2min=1050;
            float esc2max=1850;
            float esc2scale = (esc2max - esc2min) / 2.0;
            float esc2off = (esc2max + esc2min) / 2.0;
            
            m2_servoOut  = (int)(driveVal*esc2scale)+esc2off;
            break;
        case 3:
            float esc3min=1258;
            float esc3max=1770;
            float esc3off = (esc3max + esc3min) / 2.0;
            float esc3scale = (esc3max - esc3min) / 2.0;
            
            m3_servoOut  = (int)(driveVal*esc3scale)+esc3off;
            break;
        case 4:
            float esc4min=1050;
            float esc4max=1850;
            float esc4off = (esc4max + esc4min) / 2.0;
            float esc4scale = (esc4max - esc4min) / 2.0;
            
            m4_servoOut  = (int)(driveVal*esc4scale)+esc4off;
            break;
    }
}

//scales the input control driveVal -1.0 to 1.0 to the Electronic Speed Controllers
// tested parameters (reversible)
void driveESCrev(int esc, float driveVal){
    
    switch(esc){
        case 1:
            float esc1min=1050;
            float esc1max=1900;
            float esc1off = (esc1max + esc1min) / 2.0;
            float esc1scale = (esc1max - esc1min) / 2.0;
            
            testvalf = (int)(driveVal*(esc1scale/2.0))+esc1off;
            m1_servoOut = (int)(driveVal*(esc1scale/2.0))+esc1off;
            break;
        case 2:
            float esc2min=1050;
            float esc2max=1850;
            float esc2scale = (esc2max - esc2min) / 2.0;
            float esc2off = (esc2max + esc2min) / 2.0;
            
            m2_servoOut  = (int)(driveVal*(esc2scale/2.0))+esc2off;
            break;
        case 3:
            float esc3min=1258;
            float esc3max=1770;
            float esc3off = (esc3max + esc3min) / 2.0;
            float esc3scale = (esc3max - esc3min) / 2.0;
            
            m3_servoOut  = (int)(driveVal*(esc3scale/2.0))+esc3off;
            break;
        case 4:
            float esc4min=1050;
            float esc4max=1850;
            float esc4off = (esc4max + esc4min) / 2.0;
            float esc4scale = (esc4max - esc4min) / 2.0;
            
            m4_servoOut  = (int)(driveVal*(esc4scale/2.0))+esc4off;
            break;
    }
}

//Initialize the Inertial Measurement Unit (BNO-055)
void bno_init(void){
    if(bno.check()){
        pc.printf("BNO055 connected\r\n");
        bno.reset();
        wait(.5);
        bno.setmode(OPERATION_MODE_CONFIG);
        bno.SetExternalCrystal(1);
        //bno.set_orientation(1);
        
        // page 20-21 BNO055 Datasheet
        //bno.setmode(OPERATION_MODE_IMUPLUS);    //No Magnetometer and relative
        bno.setmode(OPERATION_MODE_NDOF);  //Uses magnetometer
        //bno.setmode(OPERATION_MODE_NDOF_FMC_OFF);   //no magnetometer
        bno.set_angle_units(RADIANS);
        
        //write calibration data
        //for(int i=0;i<22;i++){
        //    bno.calibration[i] = calib_local[i];
        //}
        //bno.write_calibration_data();
    }
    else{
        pc.printf("BNO055 NOT connected\r\n Program Trap.");
        while(1);
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
    pc.printf("pitchp - set the Proportional gain for the pitch controller\r\n");
    pc.printf("pitchi - set the Integral gain for the pitch controller\r\n");
    pc.printf("pitchd - set the Derivative gain for the pitch controller\r\n");
    pc.printf("pitchsp - set the pitch setpoint in radians\r\n");
    pc.printf("pitchsign - set the sign of the pitch output\r\n");
    wait(.02);
    pc.printf("yawp - set the Proportional gain for the yaw controller\r\n");
    pc.printf("yawi - set the Integral gain for the yaw controller\r\n");
    pc.printf("yawd - set the Derivative gain for the yaw controller\r\n");
    pc.printf("yawsp - set the yaw setpoint in radians\r\n");
    pc.printf("yawsign - set the sign of the yaw output\r\n");
    wait(.02);
    pc.printf("depthp - set the Proportional gain for the depth controller\r\n");
    pc.printf("depthi - set the Integral gain for the depth controller\r\n");
    pc.printf("depthd - set the Derivative gain for the depth controller\r\n");
    pc.printf("depthsp - set the depth setpoint in meters\r\n");
    pc.printf("depthsign - set the sign of the depth output\r\n");
    wait(.02);
    pc.printf("thrustsp - set the thrust setpoint\r\n");
    pc.printf("thrustsign - set the sign of the thrust output\r\n");
    wait(.02);
    pc.printf("driveesc - output servo pulse for esc scaled from 0.0-1.0\r\n");
    pc.printf("driveescrev - output servo pulse for reversible esc scaled from 0.0-1.0\r\n");
    pc.printf("SPACEBAR - SPACEBAR turns off all servo pulse outputs (stops thrusters)\r\n");
    
    //wait to receive character before exiting
    while(!pc.readable()){                                 
        char c = pc.getc();
            return;
    }
}
// ---------------- MAIN ------------------------------ 
int main() {
    char cmd_str[30];
    
    wait(.5);       //allow time for voltages to settle after micro awakens
    pc.baud(115200);
    
    pc.printf("%s\r\n", __FILE__);  //serial transmit of file name
    wait(.5);
    bno_init();     //Initialize the BNO055 IMU

    tickerPulse.attach(&heartbeat, .02);    //run pitch controller every 20 milliseconds
    
    //Initialize the depth sensor
    depthsensor.MS5837Reset();
    depthsensor.MS5837Init();
    
    //Initialize the controllers
    init_pitchController();
    init_yawController();
    init_depthController();
    //Start the actuators updates
    ticketUpdateActuators.attach(&updateActuators, .02);
    
    while(1) {
        bno.get_angles();
        yaw_cor = bno.euler.yaw * YAW640_RATIO; //correct the yaw for radian measurement
        bno.get_calib();
        //bno.get_accel();
        
        int sys_stat = (bno.calib >> 6) & 0x03;
        int gyr_stat = (bno.calib >> 4) & 0x03;
        int acc_stat = (bno.calib >> 2) & 0x03;
        int mag_stat = bno.calib & 0x03;
        
        //pc.printf("T:%02d:%02d:%02d:%02d:%03d ", runTime.day, runTime.hour, runTime.min, runTime.sec, runTime.ms);
        pc.printf("%02Xh %.2f %.2f %.2f ", bno.calib, bno.euler.roll, bno.euler.pitch, yaw_cor);
        pc.printf("pitch_co=%.3f yaw_co=%.3f ", pitch_co, yaw_co);
        //pc.printf("Roll=%6.3f Pitch=%6.3f  Yaw=%6.3f ", -bno.euler.roll, bno.euler.pitch, bno.euler.yaw);
        //pc.printf("STAT= S%1d G%1d A%1d M%1d ", sys_stat,gyr_stat,acc_stat,mag_stat);
        //pc.printf("Accel X = %.3f Accel Y = %.3f Accel Z = %.3f ", bno.accel.x, bno.accel.y, bno.accel.z);
        pc.printf("yaw_temp=%.3f yaw_err=%.3f yaw_sign=%.3f ", yaw_temp, yaw_error, yaw_sign);
        pc.printf("yaw_P=%.3f yaw_I=%.3f yaw_D=%.3f ", yaw_P, yaw_I, yaw_D);
        pc.printf("\r\n");
        
        wait(.05);
        
        if(pc.readable()){            //if data in receive buffer
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
                
                wait(1);
                bno.reset();
                wait(.5);
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
            //change the pitch proportional gain
            if(!strcmp(cmd_str, "pitchp")){
                float Pk_temp;
                pc.printf("Set pitch proportional gain (currently %.3f)\r\n", pitch_Pk);
                pc.scanf("%f", &Pk_temp);
                pitch_Pk = Pk_temp;
                
                pid_pitch.setTunings(pitch_Pk, pitch_Ik, pitch_Dk);
                
                pc.printf("Pitch Proportional gain is %.3f\r\n", pitch_Pk);
                pc.printf("\r\n\r\nPress key to continue.\r\n");
                while(!pc.readable());
                char c = pc.getc();
                c=0;
            }
            
            //change the integral gain for the pitch controller
            if(!strcmp(cmd_str, "pitchi")){
                float Ik_temp;
                pc.printf("Set pitch integral gain (currently %.3f)\r\n", pitch_Ik);
                pc.scanf("%f", &Ik_temp);
                pitch_Ik = Ik_temp;
                
                pid_pitch.setTunings(pitch_Pk, pitch_Ik, pitch_Dk);
                
                pc.printf("Pitch integral gain is %.3f\r\n", pitch_Ik);
                pc.printf("\r\n\r\nPress key to continue.\r\n");
                while(!pc.readable());
                char c = pc.getc();
                c=0;
            }            
            
            //change the derivative gain for the pitch controller
            if(!strcmp(cmd_str, "pitchd")){
                float Dk_temp;
                pc.printf("Set pitch derivative gain (currently %.3f)\r\n", pitch_Dk);
                pc.scanf("%f", &Dk_temp);
                pitch_Dk = Dk_temp;
                
                pid_pitch.setTunings(pitch_Pk, pitch_Ik, pitch_Dk);
                
                pc.printf("Pitch derivative gain is %.3f\r\n", pitch_Dk);
                pc.printf("\r\n\r\nPress key to continue.\r\n");
                while(!pc.readable());
                char c = pc.getc();
                c=0;
            }            
            
            //change the set point on the pitch controller
            if(!strcmp(cmd_str, "pitchsp")){
                pc.printf("Set pitch (currently %.3f)\r\n", pitch_sp);
                pc.scanf("%f", &pitch_sp);
                
                if(pitch_sp < PITCH_MIN_SP){
                    pitch_sp = PITCH_MIN_SP;
                    pc.printf("Pitch Entered Exceeds min value of %.3f\r\n", PITCH_MIN_SP);
                }
                if(pitch_sp > PITCH_MAX_SP){
                    pitch_sp = PITCH_MAX_SP;
                    pc.printf("Pitch Entered Exceeds max value of %.3f\r\n", PITCH_MAX_SP);
                }
                
                pc.printf("Pitch is %.3f\r\n", pitch_sp);
                
                pc.printf("\r\n\r\nPress key to continue.\r\n");
                while(!pc.readable());
                char c = pc.getc();
                c=0;
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
            //change the yaw proportional gain
            if(!strcmp(cmd_str, "yawp")){
                float Pk_temp;
                pc.printf("Set yaw proportional gain (currently %.3f)\r\n", yaw_Pk);
                pc.scanf("%f", &Pk_temp);
                yaw_Pk = Pk_temp;
                
                pid_yaw.setTunings(yaw_Pk, yaw_Ik, yaw_Dk);
                
                pc.printf("Yaw Proportional gain is %.3f\r\n", yaw_Pk);
                pc.printf("\r\n\r\nPress key to continue.\r\n");
                while(!pc.readable());
                char c = pc.getc();
                c=0;
            }
            
            //change the integral gain for the yaw controller
            if(!strcmp(cmd_str, "yawi")){
                float Ik_temp;
                pc.printf("Set yaw integral gain (currently %.3f)\r\n", yaw_Ik);
                pc.scanf("%f", &Ik_temp);
                yaw_Ik = Ik_temp;
                
                pid_yaw.setTunings(yaw_Pk, yaw_Ik, yaw_Dk);
                
                pc.printf("yaw integral gain is %.3f\r\n", yaw_Ik);
                pc.printf("\r\n\r\nPress key to continue.\r\n");
                while(!pc.readable());
                char c = pc.getc();
                c=0;
            }            
            
            //change the derivative gain for the yaw controller
            if(!strcmp(cmd_str, "yawd")){
                float Dk_temp;
                pc.printf("Set yaw derivative gain (currently %.3f)\r\n", yaw_Dk);
                pc.scanf("%f", &Dk_temp);
                yaw_Dk = Dk_temp;
                
                pid_yaw.setTunings(yaw_Pk, yaw_Ik, yaw_Dk);
                
                pc.printf("yaw derivative gain is %.3f\r\n", yaw_Dk);
                pc.printf("\r\n\r\nPress key to continue.\r\n");
                while(!pc.readable());
                char c = pc.getc();
                c=0;
            }            
            
            //change the set point on the yaw controller
            if(!strcmp(cmd_str, "yawsp")){
                pc.printf("Set yaw (currently %.3f)\r\n", yaw_sp);
                pc.scanf("%f", &yaw_sp);
                
                if(yaw_sp < YAW_MIN_SP){
                    yaw_sp = YAW_MIN_SP;
                    pc.printf("yaw Entered Exceeds min value of %.3f\r\n", YAW_MIN_SP);
                }
                if(yaw_sp > YAW_MAX_SP){
                    yaw_sp = YAW_MAX_SP;
                    pc.printf("yaw Entered Exceeds max value of %.3f\r\n", YAW_MAX_SP);
                }
                
                pc.printf("Yaw is %.3f\r\n", yaw_sp);
                
                pc.printf("\r\n\r\nPress key to continue.\r\n");
                while(!pc.readable());
                char c = pc.getc();
                c=0;
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
            //change the yaw proportional gain
            if(!strcmp(cmd_str, "depthp")){
                float Pk_temp;
                pc.printf("Set depth proportional gain (currently %.3f)\r\n", depth_Pk);
                pc.scanf("%f", &Pk_temp);
                depth_Pk = Pk_temp;
                
                pid_depth.setTunings(depth_Pk, depth_Ik, depth_Dk);
                
                pc.printf("depth Proportional gain is %.3f\r\n", depth_Pk);
                pc.printf("\r\n\r\nPress key to continue.\r\n");
                while(!pc.readable());
                char c = pc.getc();
                c=0;
            }
            
            //change the integral gain for the depth controller
            if(!strcmp(cmd_str, "depthi")){
                float Ik_temp;
                pc.printf("Set depth integral gain (currently %.3f)\r\n", depth_Ik);
                pc.scanf("%f", &Ik_temp);
                depth_Ik = Ik_temp;
                
                pid_depth.setTunings(depth_Pk, depth_Ik, depth_Dk);
                
                pc.printf("depth integral gain is %.3f\r\n", depth_Ik);
                pc.printf("\r\n\r\nPress key to continue.\r\n");
                while(!pc.readable());
                char c = pc.getc();
                c=0;
            }            
            
            //change the derivative gain for the depth controller
            if(!strcmp(cmd_str, "depthd")){
                float Dk_temp;
                pc.printf("Set depth derivative gain (currently %.3f)\r\n", depth_Dk);
                pc.scanf("%f", &Dk_temp);
                depth_Dk = Dk_temp;
                
                pid_depth.setTunings(depth_Pk, depth_Ik, depth_Dk);
                
                pc.printf("depth derivative gain is %.3f\r\n", depth_Dk);
                pc.printf("\r\n\r\nPress key to continue.\r\n");
                while(!pc.readable());
                char c = pc.getc();
                c=0;
            }            
            
            //change the set point on the depth controller
            if(!strcmp(cmd_str, "depthsp")){
                pc.printf("Set depth (currently %.3f)\r\n", depth_sp);
                pc.scanf("%f", &depth_sp);
                
                if(depth_sp < DEPTH_MIN_SP){
                    depth_sp = DEPTH_MIN_SP;
                    pc.printf("depth Entered Exceeds min value of %.3f\r\n", DEPTH_MIN_SP);
                }
                if(depth_sp > DEPTH_MAX_SP){
                    depth_sp = DEPTH_MAX_SP;
                    pc.printf("depth Entered Exceeds max value of %.3f\r\n", DEPTH_MAX_SP);
                }
                
                pc.printf("Depth is %.3f\r\n", depth_sp);
                
                pc.printf("\r\n\r\nPress key to continue.\r\n");
                while(!pc.readable());
                char c = pc.getc();
                c=0;
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
                //turn off all pulsewidths to 1000 (stops motors faster then 0
                m1_servoOut=1500;
                m2_servoOut=1500;
                m3_servoOut=1500;
                m4_servoOut=1500;
                
                int quitFlag=0;
                while(!quitFlag){
                    pc.printf("Enter servo output to Drive (1-4): ");
                    int servoNum=0;
                    pc.scanf("%d", &servoNum);
                    pc.printf("Servo %d entered\r\n Enter micro-second pulse width value:", servoNum);
                    int pwusVal=0;
                    pc.scanf("%d", &pwusVal);
                    pc.printf("\r\npulse width of %d micro-seconds entered\r\n", pwusVal);
                    
                    if(servoNum>0 && servoNum<5){
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
                        }
                    }
                    else{
                        pc.printf("Invalid Servo number entered!Exiting Now!\r\n");
                        wait(3);
                        quitFlag=1;    
                    }    
                }
            }//servoout command
            
            //drive the ESC to calibrated servo pulse outputs (non-reversible ESC)
            if(!strcmp(cmd_str, "driveesc")){
                //turn off all pulsewidths (disable all ESCs?)
                m1_servoOut=0;
                m2_servoOut=0;
                m3_servoOut=0;
                m4_servoOut=0;
                
                int quitFlag=0;
                while(!quitFlag){
                    pc.printf("Enter servo output to Drive (1-4): ");
                    int servoNum=0;
                    pc.scanf("%d", &servoNum);
                    pc.printf("Servo %d entered\r\n Enter pulse output scaled from (0.0-1.0):", servoNum);
                    float pwscaledVal;
                    
                    //keep in bounds
                    if(pwscaledVal < 0.0)
                        pwscaledVal = 0.0;
                    if(pwscaledVal > 1.0)
                        pwscaledVal = 1.0;
                        
                    pc.scanf("%d", &pwscaledVal);
                    pc.printf("\r\npulse width of %.3f entered\r\n", pwscaledVal);
                                        
                    driveESC(servoNum, pwscaledVal);
                }
            }//driveESC command
            
            //drive the ESC to calibrated servo pulse outputs (reversible ESC)
            if(!strcmp(cmd_str, "driveescrev")){
                ticketUpdateActuators.dettach();
                //turn off all pulsewidths (disable all ESCs?)
                m1_servoOut=0;
                m2_servoOut=0;
                m3_servoOut=0;
                m4_servoOut=0;
                
                int quitFlag=0;
                while(!quitFlag){
                    pc.printf("Enter servo output to Drive (1-4): ");
                    int servoNum=0;
                    pc.scanf("%d", &servoNum);
                    pc.printf("Servo %d entered\r\n Enter pulse output scaled from (-1.0 to 1.0):", servoNum);
                    float pwscaledVal;
                    
                    //keep in bounds
                    if(pwscaledVal < -1.0)
                        pwscaledVal = -1.0;
                    if(pwscaledVal > 1.0)
                        pwscaledVal = 1.0;
                        
                    pc.scanf("%d", &pwscaledVal);
                    pc.printf("\r\npulse width of %.3f entered\r\n", pwscaledVal);
                                        
                    driveESCrev(servoNum, pwscaledVal);
                }
            }//driveESCrev
            
            // ' ' (SPACEBAR) stops all servo outputs
            if(!strcmp(cmd_str, " ")){
                //turn off all pulsewidths (disable all ESCs?)
                m1_servoOut=0;
                m2_servoOut=0;
                m3_servoOut=0;
                m4_servoOut=0;
                
                while(pc.readable()){
                    char c = pc.getc();
                }
                pc.printf("\r\n\r\nPress key to continue.\r\n");
                while(!pc.readable());
                char c = pc.getc();
                c=0;                
            }
            
            // start the main combined actuator controller
            if(!strcmp(cmd_str, "startcon")){                
                ticketUpdateActuators.attach(&updateActuators, .02);
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
                if(thrust < -THRUST_MIN_SP){
                    thrust = THRUST_MIN_SP;
                }
                if(thrust >= THRUST_MAX_SP){
                    thrust = THRUST_MAX_SP;
                }
                
                pc.printf("Thrust is %.3f\r\n", thrust);
            }            
        }//if pc.readable()
        
    }//while(1)
}//main()
