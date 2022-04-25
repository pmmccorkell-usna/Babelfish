#include "mbed.h"
#include "BNO055.h"     //imu
#include "PID.h"
#define PI_math           3.14159265358979323846

//Setup USB Serial
Serial pc(USBTX, USBRX);
int baudrate = 115200;

//Setup BNO055 and MS5837 over I2C
I2C i2c(p28,p27);
DigitalOut pwr_on(p30);
BNO055 imu(i2c,p8);

int wait_main=20;       //ms to wait in main, and associated, loops

/*
// IO pins used with oscope to observe and gather timing data of the program itself
DigitalOut function_timer(p5);      //el logic + depth and pitch controllers
DigitalOut function_timer2(p6);     //sensor update and data stream
DigitalOut function_timer3(p7);     //az logic + heading and speed controllers
DigitalOut function_timer4(p8);     //high edge when a command is read and completed execution. 
                                        //Pair with RasPi GPIO and use Oscope to time how long it 
                                        //takes to send, receive, and execute commands.
*/

DigitalIn leak_detect(p11);
DigitalOut leak_light(LED1);

// ESC specs data
double esc_freq=400;     //Standard servo rate -> 400Hz
double base_pw=1.5;     //ms
double null_pw=0.0;
double esc_period=(1000/esc_freq);  //ms
double esc_range_spec=.4;   // 400ms, [1.1,1.9]ms pw per data sheet
double esc_range_scale=1.0; //only use x% of full range
double esc_range=esc_range_spec*esc_range_scale;   
double min_thruster_bias=0.028;  //deadzone around 1.5ms where ESC will not operate
double pw_tolerance=0.00001;

volatile float ticker_rate= 0.02;   //Time between ticker calls in seconds.
float target_time=5;    //Ideal timeframe to reach desired heading, depth, pitch using only Kp.
float target_time_multiple=2;   //Ki will max out PWM in "factor * target_time".

//
//Azimuth controllers
//
Ticker tickerAzThrusters;
//degrees of acceptable heading tolerance
double az_tolerance=2;
// PID gains for heading
volatile float heading_Kp =0.0008333f;
volatile float heading_Ki =0.0f;
volatile float heading_Kd =0.0f;        
PID pid_heading(heading_Kp,heading_Ki,heading_Kd,ticker_rate,az_tolerance);
// PID gains for speed
// volatile float speed_Kp=1.0f;
// volatile float speed_Ki=0.0f;
// volatile float speed_Kd=0.0;
// PID pid_speed(speed_Kp, speed_Ki, speed_Kd, ticker_rate, speed_tolerance);

//instantiate globals for sensor updates and data stream
uint16_t ready_prefix = 0x0000;
uint16_t horizon_prefix=0xff00;
uint16_t ready_data = 0x0000;
uint16_t heading = 0xffff;
uint16_t pitch = 0xffff;
char readline[100];

//instantiate globals for flags and function indication
int command_available=1;
int call_threads_available=1;
int logic_available=1;
int manual_mode=0;
int zero_set=0;
int horizon_count=0;
int event_horizon_flag=0;

//instantiate goto position globals
//-1 indicates no setting
int persistent_heading=-1;
int persistent_speed=-1;
int persistent_offset=-1;

BNO055_ID_INF_TypeDef       bno055_id_inf;
BNO055_EULER_TypeDef        euler_angles;
//BNO055_QUATERNION_TypeDef   quaternion;
//BNO055_LIN_ACC_TypeDef      linear_acc;
//BNO055_GRAVITY_TypeDef      gravity;
//BNO055_TEMPERATURE_TypeDef  chip_temp;


//-----THRUSTER CLASS BEGIN-----//
//Thruster class to instantiate individual thrusters.
class Thruster {
    public:
        Thruster(PinName pin, float dir);
        void setEvent();
        void clearEvent();
        int available();
        void set_period(double thruster_time);
        void set_pw(double thruster_pw);
        double get_pw();
        double get_speed();
        uint32_t thruster_data();
        void set_speed(double pntr);
    protected:
        PwmOut _pwm;
        PinName _pin;
        float _d;
        int _lock;
        int _available;
        double _base_pw, _period;
};

//Instantiation accepts PWM pin and direction
//Direction is -1 or 1. 1 for normal, -1 if blade reversed.
Thruster::Thruster(PinName pin, float dir) : _pwm(pin), _d(dir) {
    _lock=0;
    _available=1;
    _pin=pin;
    _base_pw=1.5;
    set_pw(_base_pw);
    _period=2.5;
    set_period(_period);
    //pc.printf("Thruster: %f\r\n",d);
}

//Sets Event for Emergency Stop and sets lockout to set_speed() function.
void Thruster::setEvent() {
    _lock=1;
    set_pw(_base_pw);
}

//Clears Event for Emergency Stop of thruster and removes lockout from set_speed() function.
void Thruster::clearEvent() {
    _lock=0;
}

//Returns whether set_speed() function is available, or currently in use.
int Thruster::available() {
    return _available;
}

//Set PWM period in ms.
void Thruster::set_period(double thruster_time) {
    _period=thruster_time;
    _pwm.period(_period/1000);
}

//Set PWM pulsewidth in ms
void Thruster::set_pw(double thruster_pw) {
    double s_pw=(thruster_pw/1000);
    pc.printf("log: set_pw: %f\r\n",s_pw);
    _pwm.pulsewidth(s_pw);
}

//Returns PWM pulsewidth in ms.
double Thruster::get_pw() {
    //read duty cycle times period
    double g_pw = (_pwm.read()*_period);
    //pc.printf(" get_pw: %f, ",g_pw);
    return g_pw;
}

//Returns PWM output relative to 1.5ms.
double Thruster::get_speed() {
    double g_speed = (get_pw()-_base_pw);
    //pc.printf("get_speed: %f, ",g_speed);
    return g_speed;
}

//formats PWM as an 2 uint16_t joined to make uint32_t for serial data streaming
//MSB uint16_t indicates direction, 0 for positive, 1 for negative.
//LSB uint16_t is 10ms resolution of PWM 
uint32_t Thruster::thruster_data() {
    double speed=get_speed();
    uint32_t dir=0x0;
    uint32_t data=0x0;
    if (speed<0) dir =0x00010000;
    data=static_cast<unsigned int>(abs(int(speed*100000)));
    data=data+dir;
    return data;
}

//Accepts adjustment to pw [-500,500] ms that is added to 1.5ms
void Thruster::set_speed(double speed_pw) {
    if (_lock==1) {
        set_pw(_base_pw);
    }
    else {
        double tolerance_pw=0.001;
        double target_pw=(_d*speed_pw)+_base_pw;
        double current_pw=get_pw();
        double diff_pw=abs(target_pw-current_pw);
        if (diff_pw>tolerance_pw) set_pw(target_pw);
    }
}
//-----THRUSTER CLASS END-----//

// Instantiate thrusters.
Thruster fwd_port(p21,1);
Thruster fwd_star(p22,-1);
//Thruster steadystate(p23,1); //for test purposes, to keep ESC from making LOUD NOISES
Thruster aft_port(p24,-1);
Thruster aft_star(p25,1);

//Function to check for water leak. Open is good, short is bad.
void leak_data() {
    leak_detect.read();
    if (leak_detect==1) {
        ready_data=(ready_data | 0x0800);
    }
    leak_light=leak_detect;
}

//Function to format Gain values into 3 significant digits and exponent value.
// ie 0x abc * 10^(d-9) -> 0x abcd
uint16_t k_data(float k_val) {
    
    // Static lookup table for 10^11 to 10^-4.
    // Arrived at for range to get 3 significant digits for values from 10^-9 to 10^6, respectively.
    static double pow10[16] = {
        100000000000, 10000000000, 1000000000, 100000000, //[0,4]
        10000000, 1000000, 100000, 10000, //[5,8]
        1000, 100, 10, 1, //[9,12]
        0.1, 0.01, 0.001, 0.0001, //[13,16]
    };
    
    // Find where significant digits start and get the exponent value.
    int exponent_value = floor(log10(k_val));

    // Form scalar value of 3 significant digits.
    int sig_val = floor((k_val*pow10[exponent_value+9])+0.5);

    // Shift exponent so that 10^-9 starts at 0.
    exponent_value+=9;

    // Move Scalar left 1 full hex, and append exponent to the end.
    uint16_t return_val=(sig_val<<4)+(exponent_value);

    //Return the formed hex value.
    return return_val;
}

//Data function pulls data from BNO and sends over serial
//Timed using DigitalOut and Oscope. 
//  At baud 115200, averaged 5 times over 256, +pulsewidth 11.1 - 13.3ms.
//  At baud 921600, averaged over 256, +pw 4.1 - 5.5ms
//  In water, 921600 induced serial comm errors
//  Variance is due to MS5837 pressure sensor. Includes waits of 2-4ms.
void az_data() {
    //function_timer2=1;

    leak_data();
    uint32_t k=0x1234abcd;

    // if (logic_available==1) ready_data=(ready_data&0xfbff);
    // else ready_data=(ready_data|0x0400);
    if (call_threads_available==1) ready_data=(ready_data & 0xfdff);
    else ready_data=(ready_data | 0x0200);
    // if (command_available==1) ready_data=(ready_data&0xfeff);
    // else ready_data=(ready_data | 0x0100);
    if (zero_set==1) ready_data=(ready_data|0x0008);
    else ready_data=(ready_data&0xfff7);

    //Instantiate status array of 7 32-bit words.
    //First 16 bits of each 32-bit word are Identifiers for RasPi to correctly assign the trailing 16 bits of data.
    uint32_t status[14]={0};
    uint32_t ready=ready_prefix;
    ready=(ready<<16)|ready_data;

    //word 0: Key
        //Used to ensure Pi and Mbed are on same page.
    status[0]=k;

    //word 1: Status information.
        //0xffff acts as prefix to identify Status for RasPi.
        //Last 3 bits (from right) are current position (POS[0-7]). See BNO datasheet.
        //4th bit (from right) is RH turn motors enabled.
        //5th bit (from right) is LH turn motors enabled.
    status[1]=ready;

    //word 2: Calibration.
        //0xc000 acts as prefix to identify Cal for RasPi.
    status[2]=0xc0000000+imu.read_calib_status();

    //Get Euler data from BNO.
    imu.get_Euler_Angles(&euler_angles);
    
    //word 3 is Heading.
        //0xc100 acts as prefix to identify Heading for RasPi. 
    uint16_t h = euler_angles.h;
    heading=h;
    status[3]=0xc1000000+h;

    //Offset calculation: 360*16bit resolution = 5760 -> converted to hex = 0x1680
    int offset=0x1680;

    //word 4 is Roll.
        //0xc300 acts as prefix to identify Roll for RasPi.
        //BNO sends Roll and Pitch as +/- 180deg. Add offset of 360deg to avoid dealing with signed ints over serial.
    uint16_t r = offset + euler_angles.r;
    status[4]=0xc3000000+r;

    //word 5 is Pitch.
        //0xc500 acts as prefix to identify Pitch for RasPi.
        //BNO sends Roll and Pitch as +/- 180deg. Add offset of 360deg to avoid dealing with signed ints over serial.
    uint16_t p = offset + euler_angles.p;
    pitch=(p - (offset/2)); //only want 180deg offset
    status[5]=0xc5000000+p;
    
    //word 6 gets Depth from el_data() function.
        //0xb0 acts as prefix to identify Barometer Pressure.
    // status[6]=el_data();
    
    //Thruster speeds in 10us resolution.
    status[6]=((fwd_port.thruster_data() & 0x00ffffff) | 0xf1000000);
    status[7]=((fwd_star.thruster_data() &0x00ffffff) | 0xf2000000);
    status[8]=((aft_port.thruster_data() & 0x00ffffff) | 0xf3000000);
    status[9]=((aft_star.thruster_data() & 0x00ffffff) | 0xf4000000);

    //Gains in format: ABC * 10^(D-9), similar to resistor color bands but starting at 10^-9 rather than 10^0.
    //Value is sent in Hex, but the 3bit scalar, ABC, is decimal [0,999]. Exponent component, D, may be hex to obtain range of [10^-9,10^6]
    
    //Heading Gains
    status[10]= 0xd1100000 + k_data(heading_Kp); //Kp
    status[11]= 0xd1200000 + k_data(heading_Ki); //Ki
    status[12]= 0xd1300000 + k_data(heading_Kd); //Kd

    //For loop iterates through Status array to transmit 6 32-bit words over Serial with "\n" appended for Python in RasPi.
    int i;
    int l=(sizeof(status)/sizeof(uint32_t))-1;
    for (i=0; i<=l; i++) {
        pc.printf("%x\n", status[i]);
  }
  //function_timer2=0;
}

//reset all controller set points and zero out any accumulated integral gains to ESCs
void stop_all_persistent() {
    persistent_heading=-1;
    persistent_speed=-1;
    pid_heading.clear();
    // pid_speed.reset();
}

//When bad things are happening.
void EventHorizon() {
    event_horizon_flag=1;
    stop_all_persistent();
    horizon_count++;
    pc.printf("log: EventHorizon called, count: %i\r\n",horizon_count);
    //setEvent() method locks out Thruster class set_speed() function
    //  and sets PWM to 1.5ms.
    fwd_port.setEvent();
    fwd_star.setEvent();
    aft_port.setEvent();
    aft_star.setEvent();
    pc.printf("log: Thruster events successfully set\r\n");
    //Tells Raspi that Emergency state has been initiated.
    ready_prefix=(horizon_prefix+horizon_count);
    //Wait some time during which Thruster set_speed() functions are locked out.
    for (int i=0; i<200; i++) {
        //Resume streaming data to RasPi during timeout period.
        az_data();
        wait_ms(wait_main);
    }
    //Clear emergency situation.
    fwd_port.clearEvent();
    fwd_star.clearEvent();
    aft_port.clearEvent();
    aft_star.clearEvent();
    pc.printf("log: Thruster events successfully cleared\r\n");
    //Set PWM to 1.5ms after emergency situation. Should have been set to 1.5ms, but double checking.
    //  For extra precaution.
    stop_all_persistent();
    //Lets Raspi know that Mbed is ready for commands again.
    ready_prefix=0xffff;
    pc.printf("log: ready status reset, mbed may resume\r\n");
    event_horizon_flag=0;
}

//Commands function handles Serial input, checks for correct syntax, and calls appropriate function(s) to execute commands.
int read_serial() {
    int i=0;
    while (pc.readable()) {
        readline[i]=pc.getc();
        pc.printf("log: input read %c at %i\r\n",readline[i],i);
        i++;
    }
    //pc.printf("i: %i\r\n",i);
    return i;
}
int look_horizon() {
    int returnval=0;
    pc.printf("log: THREAD START, horizon\r\n");
    //int length=0;
        //if (length==4) {
    char check_HORIZON[5]="STOP";
    int verified_HORIZON=0;
    for (int i=0; i<5; i++) {
        if (readline[i]==check_HORIZON[i]) verified_HORIZON++;
    }
    if (verified_HORIZON==4) {
        EventHorizon();
        returnval=1;
    }
        //}
    pc.printf("log: THREAD END, horizon\r\n");
    return returnval;
}

//TODO: for lateral movement
//double offsetController() {
//}

//Function to go to set heading.
//Timed with DigitalOut on Oscope.
//With no heading set, 28.6us.
//With heading calculations, ~32.8us
//With logging added, ~1.278ms  -> serial print statements induce significant delays
double headingController() {
    // call_threads_available=0;
    double diff=0;
    double speed=0;
    double dir=1;
    double desired_heading=persistent_heading;
    double current_heading=heading;
    current_heading=(current_heading/16);
    if (desired_heading!=-1) {
        //Calculate how far to turn in degrees.
        diff = abs(desired_heading-current_heading);
        //pc.printf("log: diff before if: %f\r\n",diff);

        //Correct for 360-0 edge cases if 'diff'erence is greater than 180.
        //Change direction and recalculate for accurate 'tolerance' comparison.
        if (diff>180.0) {
            //pc.printf("log: diff>180: %f\r\n",diff);
            if (desired_heading>180) {
                current_heading=current_heading+180;
                desired_heading=desired_heading-180;
                pc.printf("log: desired>180, desired: %f, current: %f, dir: %f, diff: %f\r\n",desired_heading,current_heading,dir,diff);
            }
            else {  //current_heading>180
                current_heading=current_heading-180;
                desired_heading=desired_heading+180;
                pc.printf("log: current>180, desired: %f, current: %f, dir: %f, diff: %f\r\n",desired_heading,current_heading,dir,diff);
            }
        }
        speed = pid_heading.process(desired_heading,current_heading);
        // pc.printf("log: des %f, cur %f, pr speed %f\r\n",desired_heading,current_heading,speed);

        //Necessary to overcome 25us deadzone around 1.5ms
        if ((abs(speed)<min_thruster_bias) and (abs(diff)>az_tolerance)) {
            if (speed<0) speed=(-1*min_thruster_bias);
            else speed=min_thruster_bias;
        }
    }
    // call_threads_available=1;
    return (speed*dir);
}

//Controller to maintain the scalar component of velocity vector for starboard front and port aft thrusters.
double cos_speedController() {
    // call_threads_available=0;
	double offset_factor=0;
	if (persistent_offset!=-1) {
		offset_factor=(((2*PI_math)/360)*persistent_offset);
	}
	offset_factor+=(PI_math/4);
	double desired_speed=persistent_speed*cos(offset_factor);
    desired_speed=(desired_speed/1000);     //convert int us to ms

    // call_threads_available=1;
    return (desired_speed);
}

//Controller to maintain the scalar component of velocity vector for port front and starboard aft thrusters.
double sin_speedController() {
    // call_threads_available=0;
	double offset_factor=0;
	if (persistent_offset!=-1) {
		offset_factor=(((2*PI_math)/360)*persistent_offset);
	}
	offset_factor+=(PI_math/4);
	double desired_speed=persistent_speed*sin(offset_factor);
    desired_speed=(desired_speed/1000);     //convert int us to ms

    // call_threads_available=1;
    return (desired_speed);
}

// Make superposition of all Controllers accessing thrusters acting on Az plane.
//  Only function that shall write to Az plane thrusters.
//  This will also deprecate call_threads() function.
void az_thruster_logic() {
    if (manual_mode==0) {
//        function_timer3=1;
        // logic_available=0;
        double heading_speed=0;
        double sin_speed=0;
		double cos_speed=0;
        double fwd_star_speed=0;
        double fwd_port_speed=0;
		double aft_star_speed=0;
		double aft_port_speed=0;
        if (persistent_speed==-1) {
			cos_speed=0;
			sin_speed=0;
		}
        if (persistent_heading==-1) heading_speed=0;
        if (persistent_heading!=-1) {
            heading_speed=headingController();
        }
        if (persistent_speed!=-1) {
            cos_speed=cos_speedController();
			sin_speed=sin_speedController();
        }
        /*
        if ((persistent_heading!=-1) and (persistent_speed!=-1)) {
            heading_speed=heading_speed*.75;
            sp_speed=sp_speed*.75;
        }
        */

        //Create Superposition of Heading and Speed
        //May need to divide these by 2 ?
        fwd_star_speed=(cos_speed - heading_speed);
		aft_port_speed=(cos_speed - heading_speed);
        fwd_port_speed=(sin_speed + heading_speed);
		aft_star_speed=(sin_speed + heading_speed);

        //Error checking to ensure PWM doesn't escape ESC parameters
        if (fwd_port_speed<(-1*esc_range)) fwd_port_speed=(-1*esc_range);
        if (fwd_star_speed<(-1*esc_range)) fwd_star_speed=(-1*esc_range);
		if (aft_star_speed<(-1*esc_range)) aft_star_speed=(-1*esc_range);
		if (aft_port_speed<(-1*esc_range)) aft_port_speed=(-1*esc_range);

        if (fwd_port_speed>esc_range) fwd_port_speed=esc_range;
        if (fwd_star_speed>esc_range) fwd_star_speed=esc_range;
		if (aft_star_speed>esc_range) aft_star_speed=esc_range;
		if (aft_port_speed>esc_range) aft_port_speed=esc_range;

        //Only write PWM if PW is appreciably different
        // double current_starboard_pw = fwd_star.get_pw();
        // double current_port_pw = fwd_port.get_pw();
        // double compare_port_pw=fabs((1.5+port_speed)-current_port_pw);
        // double compare_starboard_pw=fabs((1.5+starboard_speed)-current_starboard_pw);
        // if (compare_port_pw > pw_tolerance) {
        fwd_port.set_speed(fwd_port_speed);
		aft_port.set_speed(aft_port_speed);
            //pc.printf("log: port %f\r\n",port_speed);
        // }
        // if (compare_starboard_pw > pw_tolerance) {
        fwd_star.set_speed(fwd_star_speed);
		aft_star.set_speed(aft_star_speed);
            //pc.printf("log: stbd %f\r\n",starboard_speed);
        // }
        // logic_available=1;
//        function_timer3=0;
    }
}


//Function to change BNO position
void switch_pos(int position) {
    uint16_t ready_mask=0xfff8;
    if (position>=0 and position<8) {
        switch (position) {
            case 1:
                imu.set_mounting_position(MT_P1);
                ready_data=((ready_data & ready_mask)+0x001);
                break;
            case 2:
                imu.set_mounting_position(MT_P2);
                ready_data=((ready_data & ready_mask)+0x002);
                break;
            case 3:
                imu.set_mounting_position(MT_P3);
                ready_data=((ready_data & ready_mask)+0x003);
                break;
            case 4:
                imu.set_mounting_position(MT_P4);
                ready_data=((ready_data & ready_mask)+0x004);
                break;
            case 5:
                imu.set_mounting_position(MT_P5);
                ready_data=((ready_data & ready_mask)+0x005);
                break;
            case 6:
                imu.set_mounting_position(MT_P6);
                ready_data=((ready_data & ready_mask)+0x006);
                break;
            case 7:
                imu.set_mounting_position(MT_P7);
                ready_data=((ready_data & ready_mask)+0x007);
                break;
            case 0:
            default:
                imu.set_mounting_position(MT_P0);
                ready_data=((ready_data & ready_mask));
                break;
        }
    }
}

//Function for manual up/down, left/right controls.
//Takes current measurement of relevant sensor and adds/subtracts the "resolution" times "magnitude" 
//      to new set point, incrementally driving vehicle.
//      ie current heading 10deg, function(3,-1), new heading set point of 10+(-1*3) = 7deg
void increment_persistent(int select, int magnitude) {
    // int pitch_resolution=3;     //degrees
    // int depth_resolution=3;     //cm
    int heading_resolution=3;   //degrees
    int speed_resolution=27;     //us
    switch (select) {
        // //pitch
        // case 1:
            // persistent_pitch=((pitch/16)+(magnitude*pitch_resolution));
            // pid_pitch.clear();
            // break;
        
        // //depth
        // case 2:
            // persistent_depth=(depth+(magnitude*depth_resolution));
            // pid_depth.clear();
            // break;
            
        //heading
        case 3:
            persistent_heading=((heading/16)+(magnitude*heading_resolution));
            pid_heading.clear();
            break;
        
        //speed
        case 4:
            persistent_speed=((1000*fwd_star.get_speed())+(magnitude*speed_resolution));
            // pid_speed.clear();
            break;
		
		//offset
		case 5:
			persistent_offset=(persistent_offset+(magnitude*heading_resolution));
			break;
    }
}

float k_commands(int value, int power) {
    static float pow[10] = {
        0.00001, 0.0001, 0.001, 0.01, 0.1,  //[0,4]
        1, 10, 100, 1000, 10000 //[5,9]
    };
    float return_val=0;
    return_val = value*pow[power];
    pc.printf("log: return %f, pow %lf\r\n",return_val,pow[power]);
    return return_val;
}

//Function intakes serial commands.
void commands() {
    //pc.printf("log: commands called\r\n");
    int length=0;
    length=read_serial();
    if (length==4) {
        look_horizon();
    }
    if (length==7) {
        char input[10];
        for (int i=0; i<10; i++) {
            input[i]=readline[i];
            //pc.printf("Command thread: read %c at %i\r\n",readline[i],i);
        }
        //check_pos char array used to filter Position commands.
        char check_pos[5]="pos:";
        //Heading commands.
        char check_hea[5]="hea:";
        //Speed commands.
        char check_vel[5]="vel:";
		//Offset commands.
		char check_off[5]="off:";
        //Stop all commands.
        char check_sto[5]="sto:";
        //Reset Mbed command.
        char check_res[5]="res:";

        //Heading PID gains
        char check_hkp[5]="hkp:";
        char check_hki[5]="hki:";
        char check_hkd[5]="hkd:";

        //While loop reads Serial input into input buffer.

        //Only continue if input buffer has 7 entries, otherwise ignore input buffer. 
        //All commands from RasPi shall come in a format of 7 characters "abc:xyz" 
        //      where 'abc' is an identifying string and 'xyz' is some data/information.
    //    if (i==7) {
            //Instantiate counters that will be used to verify known commands.
        int verified_pos=0;
        int verified_hea=0;
        int verified_sto=0;
        int verified_res=0;
        int verified_vel=0;
		int verified_off=0;

        int verified_hkp=0;
        int verified_hki=0;
        int verified_hkd=0;

        //While loop checks first 4 characters of input buffer and attempts to match
        //      against known commands.
        for (int q=0; q<3; q++) {
            //Increment verified_pos if a match is found between Serial input buffer
            //      and Position command format.
            if (input[q]==check_pos[q]) {
                verified_pos++;
                pc.printf("pos %c at %i\r\n",input[q],q);
            }
            //Increment verified_hea if a match is found between Serial input buffer
            //      and Heading command format.
            if (input[q]==check_hea[q]) {
                //pc.printf("hea %c at %i\r\n",input[q],q);
                verified_hea++;
            }
            if (input[q]==check_sto[q]) {
                //pc.printf("sto %c at %i\r\n",input[q],q);
                verified_sto++;
            }
            if (input[q]==check_res[q]) {
                //pc.printf("res %c at %i\r\n",input[q],q);
                verified_res++;
            }
            if (input[q]==check_vel[q]) {
                verified_vel++;
                //pc.printf("vel %c at %i\r\n",input[q],q);
            }
			if (input[q]==check_off[q]) {
				verified_off++;
			}
            if (input[q]==check_hkp[q]) {
                verified_hkp++;
            }
            if (input[q]==check_hki[q]) {
                verified_hki++;
            }
            if (input[q]==check_hkd[q]) {
                verified_hkd++;
            }
        }

        //If first 4 characters from Serial input buffer match Position command format,
        //      execute "switch_pos" function.
        if (verified_pos==3) {
            int change=(input[6]-'0');
            switch_pos(change);
        }
        if (verified_sto==3) {
            //pc.printf("log: stop command received\r\n");
            stop_all_persistent();
            //pc.printf("log: stop command executed\r\n");
        }
        //If first 4 characters from Serial input buffer match Heading command format,
        //      execute "motors" function.
        if (verified_hea==3) {
            //Correct for ascii '0', and reform 3digit decimal number
            int hea100=(input[4]-'0');
            int hea10=(input[5]-'0');
            int hea1=(input[6]-'0');
            int hea=(hea100*100)+(hea10*10)+(hea1*1);
            pc.printf("log: heading rx: %i\r\n",hea);
            if (hea==999) {
                persistent_heading=-1;
                pid_heading.clear();
            }
            if ((hea>=831) and (hea<=837)) {
                increment_persistent(hea10,(hea1-4));
            }
            if ((hea>=0) and (hea<=360)) {
                pid_heading.clear();
                if (event_horizon_flag==0) persistent_heading=hea;
            }
        }   
        if (verified_res==3) {
            pc.printf("log: Reset mbed received. See you on the other side.\r\n");
            //Mbed reset command.
            NVIC_SystemReset();
            //If this print statement is ever executed, reset didn't happen.
            pc.printf("log: Reset failed. The show goes on.\r\n");
        }
        // Forward/Reverse speed commands.
        if (verified_vel==3) {
            int vel100=(input[4]-'0');
            int vel10=(input[5]-'0');
            int vel1=(input[6]-'0');
            int vel=(vel100*100)+(vel10*10)+(vel1*1);
            pc.printf("log: vel rx: %i\r\n",vel);
            //999 to stop speed controller.
            if (vel==999) {
                persistent_speed=-1;
            }
            if ((vel>=841) and (vel<=847)) {
                increment_persistent(vel10,(vel1-4));
            }
            if ((vel>=0) and (vel<=800)) {
                // pid_speed.clear();
                vel=(vel-400);
                if (event_horizon_flag==0) persistent_speed=vel;
            }
        }

		//Heading offset commands.
		if (verified_off==3) {
            int off100=(input[4]-'0');
            int off10=(input[5]-'0');
            int off1=(input[6]-'0');
            int off=(off100*100)+(off10*10)+(off1*1);
            pc.printf("log: off rx: %i\r\n",off);
			//999 to reset heading offset to 0.
			if (off==999) {
				persistent_offset=-1;
			}
			if ((off>=851) and (off<=857)) {
				increment_persistent(off10,(off1-4));
			}
			if ((off>=0) and (off<=360)) {
				if (event_horizon_flag==0) persistent_offset=off;
			}
		}

        int reset_h=0;
        if (verified_hkp==3) {
            float kval=((input[4]-'0')*10)+input[5]-'0';
            heading_Kp=k_commands(kval,input[6]-'0');
            reset_h=1;
        }
        if (verified_hki==3) {
            float kval=((input[4]-'0')*10)+input[5]-'0';
            heading_Ki=k_commands(kval,input[6]-'0');
            reset_h=1;
        }
        if (verified_hkd==3) {
            float kval=((input[4]-'0')*10)+input[5]-'0';
            heading_Kd=k_commands(kval,input[6]-'0');
            reset_h=1;
        }
        if (reset_h) {
            pid_heading.set_K(heading_Kp,heading_Ki,heading_Kd);
            pc.printf("log: update Heading gains Kp %f, Ki %f, Kd %f\r\n",heading_Kp, heading_Ki, heading_Kd);
        }
    }
}

//Initialize controllers associated with Azimuth, speed and heading.
void init_AzController(void){
    // Zero out PID values.
    // pid_speed.clear();
    pid_heading.clear();
    
    // run Az controller as set in globals, in ms
    tickerAzThrusters.attach(&az_thruster_logic, ticker_rate);   
}

int main() {
    //engage plaidspeed
    pc.baud(baudrate);

    //Uncomment to derive timing using Oscope.
    //function_timer=0;
    //function_timer2=0;
    //function_timer3=0;

    //If not ready, reset BNO.
    while (imu.chip_ready() == 0) {
        do {
            pc.printf("resetting BNO\r\n");
            pwr_on=0;
            wait_ms(100);
            pwr_on=1;
            wait_ms(50);
        } while(imu.reset());
    }
    wait_ms(20);

    //If BNO is ready, set ready status indication
    if (imu.chip_ready()) {
        ready_prefix=0xffff;
    }
    switch_pos(0);  //BNO default position 1

    init_AzController();

    //Look for serial input commands and send to 'commands' function.
    //If no serial input commands, stream data.
    while(1) {
        if (pc.readable()) {
            // command_available=0;
            commands();
            //function_timer4=1;
            // command_available=1;
        }
        else {
            az_data();
        }
        wait_ms(wait_main/2);
        //function_timer4=0;
        wait_ms(wait_main/2);
    }
}

