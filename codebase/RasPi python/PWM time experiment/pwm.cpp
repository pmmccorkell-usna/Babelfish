#include "mbed.h"
#include "BNO055.h"     //imu
#include "MS5837.h"     //pressure sensor


#define OS_THREAD_OBJ_MEM 1
#define OS_THREAD_NUM 8
#define mem OS_THREAD_OBJ_MEM
/*
#if (OS_THREAD_OBJ_MEM == 0) 
#define OS_THREAD_LIBSPACE_NUM 8
#else 
#define OS_THREAD_LIBSPACE_NUM OS_THREAD_NUM 
#endif
*/

//Setup USB Serial
Serial pc(USBTX, USBRX);

//Setup BNO055 and MS5837 over I2C
I2C i2c(p28,p27);
DigitalOut pwr_on(p30);
BNO055 imu(i2c,p8);
MS5837 press_sensor(I2C_SDA,I2C_SCL,ms5837_addr_no_CS);

//instantiate global ready to 0
uint16_t ready_prefix = 0x0000;
uint16_t horizon_prefix=0xff00;
uint16_t ready_data = 0x0000;
uint16_t heading = 0xffff;      //az_data() sets value from BNO. Instantiated outside normal values.
char readline[100];
int command_available=1;
int call_threads_available=1;
int horizon_count=0;

//instantiate globals for press sensor
int sensor_rate=512;         //Oversampling Rate, see data sheet
double zero_depth=0;        // waterline
double cal_depth=0;         // x cm below waterline
double depth_setting=10;  // (zero_depth - cal_depth) / depth_setting
double current_depth=0;     //cm
int cal_set=0;
int zero_set=0;

BNO055_ID_INF_TypeDef       bno055_id_inf;
BNO055_EULER_TypeDef        euler_angles;
//BNO055_QUATERNION_TypeDef   quaternion;
//BNO055_LIN_ACC_TypeDef      linear_acc;
//BNO055_GRAVITY_TypeDef      gravity;
//BNO055_TEMPERATURE_TypeDef  chip_temp;

// ESC specs data
double esc_freq=400;     //Standard servo rate -> 400Hz
double base_pw=1.5;     //ms
double null_pw=0.0;
double esc_period=(1000/esc_freq);  //ms
//double esc_step=(30/1000);  //25us per data sheet Blue Robotics BESC-R1
//double esc_sample_freq=400;    //400Hz update frequency per data sheet
//double esc_time_ms=(1000/esc_sample_freq);
double esc_range_spec=.4;   // 400ms, [1.1,1.9]ms pw per data sheet
double esc_range=esc_range_spec*0.7;    //only use x% of full range
double e=2.71828;       //natural #

//The sigmoid curve for logistic function.
//Used for continuous PWM and softstart / softstop near end points.
double Sigmoid(double Ymax, double m, double x0, double x) {
    //Logistic_func=(Ymax/(1+(e^((-1*m)*(x-x0)))));
    double exponent=((-1*m)*(x-x0));
    double power=pow(e,exponent);
    double Logistic_func=Ymax/(1+power);
//  pc.printf("exp: %f, power:%f, ret: %f\r\n",exponent,power,Logistic_func);
    return Logistic_func;
}

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
        double _base_pw;
};

//Instantiation accepts PWM pin and direction
//Direction is -1 or 1. 1 for normal, -1 if blade reversed.
Thruster::Thruster(PinName pin, float dir) : _pwm(pin), _d(dir) {
    _lock=0;
    _available=1;
    _base_pw=1.5;
	_pin=pin;
    set_period(2.5);
    set_pw(_base_pw);
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
    double period=(thruster_time/1000);
    _pwm.period(period);
}

//Set PWM pulsewidth in ms
void Thruster::set_pw(double thruster_pw) {
    double s_pw=(thruster_pw/1000);
	pc.printf("log: set_pw: %f\r\n",s_pw);
    _pwm.pulsewidth(s_pw);
}

//Returns PWM pulsewidth in ms.
double Thruster::get_pw() {
    //2.5ms period for 400Hz
    double g_pw = (_pwm.read()*2.5);
    //pc.printf(" get_pw: %f, ",g_pw);
    return g_pw;
}

double Thruster::get_speed() {
    double g_speed = (get_pw()-_base_pw);
    //pc.printf("get_speed: %f, ",g_speed);
    return g_speed;
}

uint32_t Thruster::thruster_data() {
    double speed=get_speed();
    uint32_t dir=0x0;
    uint32_t data=0x0;
    if (speed<0) dir =0x00010000;
    data=static_cast<unsigned int>(abs(int(speed*100000)));
    data=data+dir;
    return data;
}

//Progressively change PWM pw for Thruster using Sigmoid Curve
//Accepts adjustment to pw [-500,500] ms that is added to 1.5ms
void Thruster::set_speed(double speed_pw) {
    //set _available to 0, indicating function is in use.
    _available=0;
    if (_lock==1) {
        set_pw(_base_pw);
    }
    else {
        pc.printf("set_speed called: %f\r\n", speed_pw);
        pc.printf("speed dir: %f\r\n",_d);
        double target_pw;
        target_pw=(_d*speed_pw)+_base_pw;
        double current_pw=get_pw();
        double diff_pw=abs(target_pw-current_pw);
        pc.printf("log: target: %f, current: %f, difference: %f\r\n",target_pw,current_pw,diff_pw);
        //Variables for Sigmoid Curve.
        double Center=250; //ms, 50% ramp-up at center time
        double Slope=50; //rate of ramp up
        double time_count=0; //total time in while loop
        double dwell_time=0.001; //time to wait between while loop
        double tolerance_pw=.03; //30us
        double dir=0;
        while(diff_pw>tolerance_pw) {
            current_pw=get_pw();
            diff_pw=abs(target_pw-current_pw);
            if ((target_pw-current_pw)>=0) {
                dir=(1);
            }
            if ((target_pw-current_pw)<0) {
                dir=(-1);
            }
            double factor=((diff_pw)*(Sigmoid(1,Slope,Center,time_count)));
            if (factor<=0.03) factor=0.03;
            pc.printf("log: factor: %f, dir: %f, diff_pw: %f\r\n",factor,dir,diff_pw);
            double new_pw=(current_pw+(dir*factor));
            if (_lock==0) {
				pc.printf("pwm pin: %i",_pin);
				pc.printf("pwm pin: %c",_pin);
				set_pw(new_pw);
			}
            wait_ms(dwell_time);
            time_count+=dwell_time;
        }
        pc.printf("log: set_pw: %f\r\n",target_pw);
        set_pw(target_pw);
    }
    //Indicates set_speed() function is available.
    _available=1;
}
//-----THRUSTER CLASS END-----//

// Instantiate thrusters.
Thruster port_thrust(p21,-1);
Thruster starboard_thrust(p22,1);
Thruster steadystate(p23,1); //for test purposes, to keep ESC from beeping
Thruster fore_thrust(p24,-1);
Thruster aft_thrust(p25,1);


//Function to get elevation data and send to RasPi.
uint32_t el_data() {
    double press_slope=(cal_depth-zero_depth)/depth_setting;
    //Run Barometric equations from pressure sensor.
    press_sensor.calculate();
    uint32_t temp_comp_press=press_sensor.pressure();
    double depth=(temp_comp_press-zero_depth)/press_slope;
    uint32_t depth_data=(depth*1000);
//    pc.printf("tempcomp: %u, temp depth: %f\r\n",temp_comp_press,depth);    
    //0xb0 acts as prefix to identify Barometer Pressure.
    //Pressure sensor sends pressure in range [0x3e8,0x1d4c0]. Divide by 100 for mbar.
    uint32_t el_data_comp=(0xb1000000|depth_data);
    current_depth=depth;
    return el_data_comp;
}

//Data function pulls data from BNO and sends over serial
//uint16_t az_data(uint32_t k) {
void az_data() {
    uint32_t k=0x1234abcd;

    //Instantiate status array of 7 32-bit words.
    //First 16 bits of each 32-bit word are Identifiers for RasPi to correctly assign the trailing 16 bits of data.
    uint32_t status[11]={0};
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
    status[5]=0xc5000000+p;
    
    //word 6 gets Depth from el_data() function.
        //0xb0 acts as prefix to identify Barometer Pressure.
    status[6]=el_data();
    
    status[7]=((port_thrust.thruster_data() & 0x00ffffff) | 0xf1000000);
    status[8]=((starboard_thrust.thruster_data() &0x00ffffff) | 0xf2000000);
    status[9]=((fore_thrust.thruster_data() & 0x00ffffff) | 0xf3000000);
    status[10]=((aft_thrust.thruster_data() & 0x00ffffff) | 0xf4000000);

    //For loop iterates through Status array to transmit 6 32-bit words over Serial with "\n" appended for Python in RasPi.
    int i;
    int l=(sizeof(status)/sizeof(uint32_t))-1;
    for (i=0; i<=l; i++) {
        pc.printf("%x\n", status[i]);
//        pc.printf("mem: %i\r\n",mem);
  }
}

//Functions for each motor for threading. 
//Thread calls cannot accept more than 1 argument.
//Thread calls cannot have object.method and arguments.
//Thereby, these functions are necessary: 
// 1. Start new Thread
// 2. Call the Thruster object.method(arg) within this thread
// 3. ???
void Stbd_thread(double *d_pntr) {
    pc.printf("log: Stbd thread\r\n");
    double pw=*d_pntr;
    starboard_thrust.set_speed(pw);
}
void Port_thread(double *d_pntr) {
    pc.printf("log: Port thread\r\n");
    double pw=*d_pntr;
    port_thrust.set_speed(pw);
}
void Fwd_thread (double *d_pntr) {
    pc.printf("log: Fore thread\r\n");
    double pw=*d_pntr;
    fore_thrust.set_speed(pw);
}
void Aft_thread(double *d_pntr) {
    pc.printf("log: Aft thread\r\n");
    double pw=*d_pntr;
    aft_thrust.set_speed(pw);
}

//Function to create new threads for motor logic
int call_threads(int select, double dir=1, double speed=0) {
	Timer t;
	int i=0;
	int pw_set=0;
	double s_pw=0;
	double check_pw=0;
    int return_val=0;
	pc.printf("log: call_threads, avail: %i\r\n",call_threads_available);
    if (call_threads_available==1) {
        call_threads_available=0;
		pc.printf("log: call_threads entered, avail: %i\r\n",call_threads_available);
        double rev_speed=(-1*speed);
        Thread thruster1;
        Thread thruster2;
        int available1;
        int available2;
        //Masking for port and starboard thruster status.
        //uint16_t ready_mask_ps=0xff8f;
        uint16_t ready_mask_ps=0xff3f;
        //Masking for fore and aft thruster status.
        //uint16_t ready_mask_fa=0xf8ff;
        uint16_t ready_mask_fa=0xffcf;
        switch (select) {
            //case 1, forwards or backwards
            case 1:
                ready_data=(ready_data&ready_mask_ps)|0x00c0;
                available1=port_thrust.available();
                available2=starboard_thrust.available();
                if (available1 && available2) {
                    if (dir==1) {
                        pc.printf("log: call_threads Fwd, %f\r\n",speed);
                        //Port and Starboard forward: 111
                        //ready_data=(ready_data&ready_mask_ps)|0x0070;
                        thruster1.start(callback(Stbd_thread, &speed));
                        thruster2.start(callback(Port_thread, &speed));
                    }
                    if (dir==-1) {
                        pc.printf("log: call_threads Rev, %f\r\n",speed);
                        //Port and Starboard reverse: 100
                        //ready_data=(ready_data&ready_mask_ps)|0x0040;
                        thruster1.start(callback(Stbd_thread, &rev_speed));
                        thruster2.start(callback(Port_thread, &rev_speed));
                    }
                }
                break;

            //case 2, turn left or right
            case 2:
                ready_data=(ready_data&ready_mask_ps)|0x00c0;
                available1=port_thrust.available();
                available2=starboard_thrust.available();
                if (available1 && available2) {
                    if (dir==1) {
                        pc.printf("log: call_threads Turn R, %f\r\n",speed);
                        //Port rev, Starboard fwd: 101
                        //ready_data=(ready_data&ready_mask_ps)|0x0050;
                        thruster1.start(callback(Stbd_thread, &speed));
                        thruster2.start(callback(Port_thread, &rev_speed));
                    }
                    if (dir==-1) {
                        pc.printf("log: call_threads Turn L, %f\r\n",speed);
                        //Port fwd, Starboard rev: 110
                        //ready_data=(ready_data&ready_mask_ps)|0x0060;
                        thruster1.start(callback(Stbd_thread, &rev_speed));
                        thruster2.start(callback(Port_thread, &speed));
                    }
                }
                break;

            //case 3, Up and Down
            case 3:
                ready_data=(ready_data&ready_mask_fa)|0x0030;
                available1=fore_thrust.available();
                available2=aft_thrust.available();
                if (available1 && available2) {
                    if (dir==1) {
                        pc.printf("log: call_threads Up, %f\r\n",speed);
                        //Fore and Aft up: 111
                        //ready_data=(ready_data&ready_mask_fa)|0x0700;
                        thruster1.start(callback(Fwd_thread, &speed));
                        thruster2.start(callback(Aft_thread, &speed));
                    }
                    if (dir==-1) {
                        pc.printf("log:call_threads Down,%f\r\n",speed);
                        //Fore and Aft down: 100
                        //ready_data=(ready_data&ready_mask_fa)|0x0400;
                        thruster1.start(callback(Fwd_thread, &rev_speed));
                        thruster2.start(callback(Aft_thread, &rev_speed));
                    }
                }
                break;

            //case 4, pitch up/down
            case 4:
                ready_data=(ready_data&ready_mask_fa)|0x0030;
                available1=fore_thrust.available();
                available2=aft_thrust.available();
                if (available1 && available2) {
                        if (dir==1) {
                        pc.printf("log: call_threads Pitch Up,%f\r\n",speed);
                        //Fore up, Aft down: 110
                        //ready_data=(ready_data&ready_mask_fa)|0x0600;
                        thruster1.start(callback(Fwd_thread, &speed));
                        thruster2.start(callback(Aft_thread, &rev_speed));
                    }
                    if (dir==-1) {
                        pc.printf("log: call_threads Pitch Down,%f\r\n",speed);
                        //Fore down, Aft up: 101
                        //ready_data=(ready_data&ready_mask_fa)|0x0500;
                        thruster1.start(callback(Fwd_thread, &rev_speed));
                        thruster2.start(callback(Aft_thread, &speed));
                    }
                }
                break;
                
            //cases 5 and 6 reserved for roll should we add more thrusters.

            //case 7, Emergency Surface
            case 7:
                pc.printf("log: call_threads Emergency Surface,%f\r\n",speed);
                //Fore and Aft up: 111
                //ready_data=(ready_data&ready_mask_fa)|0x0700;
                ready_data=(ready_data&ready_mask_fa)|0x0030;
                thruster1.start(callback(Fwd_thread, &esc_range_spec));
                thruster2.start(callback(Aft_thread, &esc_range_spec));
                break;

            //case 99, Stop Fore and Aft thrusters.
            case 99:
                available1=fore_thrust.available();
                available2=aft_thrust.available();
                if (available1 && available2) {
                    pc.printf("log: Stop el thrusters,%f\r\n",speed);
                    //fore and aft thrusters stopped: 000
                    ready_data=(ready_data&ready_mask_fa);
                    thruster1.start(callback(Fwd_thread, &null_pw));
                    thruster2.start(callback(Aft_thread, &null_pw));
                }
                break;
                
            //case 0, stop az thrusters
            default:
            case 0:
                available1=port_thrust.available();
                available2=starboard_thrust.available();
                if (available1 && available2) {
                    pc.printf("log: Stop az thrusters,%f\r\n",speed);
                    //starboard and port thrusters stopped: 000
                    ready_data=(ready_data&ready_mask_ps);
                    thruster1.start(callback(Stbd_thread, &null_pw));
                    thruster2.start(callback(Port_thread, &null_pw));
                }
                break;
        }
        thruster1.join();
        thruster2.join();
//		pc.printf("log: thruster threads joined\r\n");
//		az_data();
		check_pw=(1.5+(dir*speed));
		s_pw=starboard_thrust.get_pw();
		pc.printf("log: start, %f,%f\r\n",s_pw,check_pw);
		t.start();
		while(pw_set==0) {
//			double p_pw=port_thrust.get_pw();
			s_pw=starboard_thrust.get_pw();
			pc.printf("log:%i,%f, %f\r\n",i,s_pw,check_pw);
			if (s_pw==check_pw) pw_set=1;
//			double f_pw=fore_thrust.get_pw();
//			double a_pw=aft_thrust.get_pw();
			i++;
			wait_ms(0.001);
		}
		t.stop();
		pc.printf("log: %f,stbd: %f\r\n",t.read(),s_pw);
        return_val=1;
    }
    call_threads_available=1;
    return return_val;
}

int wait_call_threads_available() {
	while (call_threads_available!=1) {
		wait_ms(5);
    }
	return 1;
}

//Function to handle vertical motor logic.
void el_thruster_logic(int desired_depth) {
    //Pull depth from pressure sensor, correct for 100bit/mBar resolution, convert to cm
//    uint8_t resolution=0x64;
//    double in_conversion=.401865;
//    double cm_conversion=1.01972;
//    double current_depth=(((el_data(0x1234abcd))/resolution)*cm_conversion);    //cm
    pc.printf("El Thruster Logic called: %i\r\n",desired_depth);
    double tolerance = 1.0;     //cm
    double diff=abs(desired_depth-current_depth);
    pc.printf("diff: %f\r\n",diff);
    while (diff>tolerance) {
        az_data();
        double dir=-1;
        double speed=0;
        if (desired_depth>current_depth) dir=(1);
        //Sigmoid data
        double center=100;   //cm
        double slope=0.5;
        speed=Sigmoid(esc_range,slope,center,diff);
        if (speed<0.03) speed=0.03;
        //Ensure existing threads are closed before restarting while loop and opening new threads.
//        int check_threads=0;
//        check_threads=
        if (call_threads_available==1) call_threads(3,dir,speed);
        int check_thrusters=0;
        int i=0;
        while (check_thrusters==0) {
            if ((aft_thrust.available()==1) and (fore_thrust.available()==1)) check_thrusters=1;
            i++;
            wait_ms(0.001);
        }
        if (check_thrusters==1) pc.printf("Threads rejoined, heading: %f, count: %i\r\n",current_depth, i);
        wait_ms(10);
        diff=abs(desired_depth-current_depth);
        pc.printf("current: %f, target: %f, diff: %f\r\n",current_depth,desired_depth,diff);
    }
}

//Function to handle lateral motor logic.
void az_thruster_logic(int desired_heading) {
    //Pull heading from BNO and correct for 16bit/deg resolution.
    double current_heading=(heading/0x10);
    //Set acceptable tolerance in deg, for heading accuracy.
    double tolerance = 2.0;
    //The natural #
    pc.printf("log:Az Thruster Logic called\r\n");
    //Error handling. Only continue for 0-360 range.
    if (desired_heading>=0 and desired_heading<=360) {
        //Calculate how far to turn in degrees.
        double diff = abs(desired_heading-(current_heading));
        //Correct for 360-0 edge cases if 'diff'erence is greater than 180.
        //Change direction and recalculate for accurate 'tolerance' comparison.
        if (diff>180) {
            if (desired_heading>180) diff=((current_heading+180)-(desired_heading-180));
            if (current_heading>180) diff=((desired_heading+180)-(current_heading-180));
        }
        while (diff > tolerance) {
            double dir=1;
            double speed=0;
            az_data();
            current_heading=(heading/0x10);
            diff=abs((desired_heading)-(current_heading));
            //Sigmoid data
            double center=80; //deg
            double slope=0.5; //rate of change
            if (diff>180) {
                dir=(-1);
                if (desired_heading>180) diff=((current_heading+180)-(desired_heading-180));
                if (current_heading>180) diff=((desired_heading+180)-(current_heading-180));
            }
            if ((desired_heading-current_heading)<0) dir=(dir*-1);
            speed=Sigmoid(esc_range,slope,center,diff);
            if (speed<0.03) speed=0.03;
            pc.printf("send speed to class: %f\r\n",speed);
//            int check_threads=0;
            if (call_threads_available==1) call_threads(2,dir,speed);
            //Ensure existing threads are closed before restarting while loop and opening new threads.
            int i=0;
            int check_thrusters=0;
            while (check_thrusters==0) {
                if ((starboard_thrust.available()==1) and (port_thrust.available()==1)) check_thrusters=1;
                i++;
                wait_ms(0.001);
            }
			pc.printf("log: time wait for threads %i us\r\n",i);
            if (check_thrusters==1) pc.printf("Threads rejoined, heading: %f, count: %i\r\n",current_heading, i);
            wait_ms(2.5);
        }
//      pc.printf("heading: %i, diff: %i\r\n",current_heading,diff);
    }
    pc.printf("thruster logic end\r\n");
}

//Function to accept desired position, program BNO,
//  and update ready word by masking non-position bits
//  and adding the new position to position bits
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

//Manual direction function allows manual control of UAV.
void manual_direction(int direction,int mode) {
    double speed_rate=0.25;         //25% speed of global max.
    double speed=(esc_range*speed_rate);    //This rate% of global% set in globals. Expected (70-90%).
    double dir;
    if (direction==1) dir = 1.0;
    else if (direction==2) dir=(-1);
    else mode=0.0;
    if (mode==0) {
        //stop az Thrusters
        if (wait_call_threads_available()==1) call_threads(0);
        //stop el Thrusters
        if (wait_call_threads_available()==1) call_threads(99);
	}
    else {
        if (wait_call_threads_available()==1) call_threads(mode,dir,speed);
    }
}

void test_direction(int mode) {
//void test_direction(int *i_pntr) {
//  int mode=*i_pntr;
    pc.printf("test_dir thread, mode:%i\r\n",mode);
    int count=0;
    manual_direction(1,mode);
    while(count<120) {
        az_data();
        if (count==50) {
            pc.printf("stop in middle of test/r/n");
            if ((mode==1) or (mode==2)) {
                if (wait_call_threads_available()==1) call_threads(0);
            }
            if ((mode==3) or (mode==4)) {
                if (wait_call_threads_available()==1) call_threads(99);
            }
        }
        if (count==70) {
            pc.printf("reverse direction\r\n");
            manual_direction(2,mode);
        }
        pc.printf("count: %i\r\n",count);
        count++;
        wait_ms(10);
    }
    //stop az thrusters
    if ((mode==1) or (mode==2)) {
        if (wait_call_threads_available()==1) call_threads(0);
    }
    //stop el thrusters
    if ((mode==3) or (mode==4)) {
        if (wait_call_threads_available()==1) call_threads(99);
    }
}

//When bad things are happening.
void EventHorizon() {
    horizon_count++;
    pc.printf("log: EventHorizon called, count: %i\r\n",horizon_count);
    //setEvent() method locks out Thruster class set_speed() function
    //  and sets PWM to 1.5ms.
    port_thrust.setEvent();
    starboard_thrust.setEvent();
    fore_thrust.setEvent();
    aft_thrust.setEvent();
	pc.printf("log: Thruster events successfully set\r\n");
    //Tells Raspi that Emergency state has been initiated.
    ready_prefix=(horizon_prefix+horizon_count);
    //Wait some time during which Thruster set_speed() functions are locked out.
    for (int i=0; i<200; i++) {
        //Resume streaming data to RasPi during timeout period.
        az_data();
        wait_ms(10);
    }
    //Clear emergency situation.
    port_thrust.clearEvent();
    starboard_thrust.clearEvent();
    fore_thrust.clearEvent();
    aft_thrust.clearEvent();
	pc.printf("log: Thruster events successfully cleared\r\n");
    //Set PWM to 1.5ms after emergency situation. Should have been set to 1.5ms, but double checking.
    // For extra precaution.
    if (wait_call_threads_available()==1) call_threads(0);
    if (wait_call_threads_available()==1) call_threads(99);
    //Tell Raspi that mbed is ready for commands again.
    ready_prefix=0xffff;
	pc.printf("log: ready status reset, mbed may resume\r\n");
	az_data();
}

int read_serial() {
    int i=0;
    while (pc.readable()) {
        readline[i]=pc.getc();
//        pc.printf("log: input read %c at %i\r\n",readline[i],i);
        i++;
    }
    //pc.printf("i: %i\r\n",i);
    return i;
}

void look_horizon(int *int_pntr) {
    int thread_select=*int_pntr;
    int quit=0;
    int length=0;
    while (quit==0) {
        if (thread_select==0) {
            quit=1;
            length=4;
        }
        if (thread_select==1) {
			pc.printf("log: horizon loop\r\n");
            length=read_serial();
            quit=command_available;
        }
//      pc.printf("look for Horizon thread called\r\n");
        if (length==4) {
            char check_HORIZON[5]="STOP";
            int verified_HORIZON=0;
            for (int i=0; i<5; i++) {
                if (readline[i]==check_HORIZON[i]) verified_HORIZON++;
            }
            if (verified_HORIZON==4) {
                EventHorizon();
            }
        }
        wait_ms(20);
    }
}

//Commands function handles Serial input, checks for correct syntax, and calls appropriate function(s) to execute commands.
void commands() {
    command_available=0;
	pc.printf("log: commands called\r\n");
    int thread_select=0;
    int length=0;
    length=read_serial();
    if (length==4) {
        look_horizon(&thread_select);
    }
    if (length==7) {
        thread_select=1;
        Thread input_thread;
        input_thread.start(callback(look_horizon,&thread_select));
        char input[10];
        for (int i=0; i<10; i++) {
            input[i]=readline[i];
            pc.printf("Command thread: read %c at %i\r\n",readline[i],i);
        }
        //check_pos char array used to filter Position commands.
        char check_pos[5]="pos:";
        //check_hea char array used to filter Go to Heading commands.
        char check_hea[5]="hea:";
        //check_dep char array used to filter Depth commands.
        char check_dep[5]="dep:";
        char check_zer[5]="zer:";
        char check_set[5]="set:";
        char check_tst[5]="tst:";
        char check_sto[5]="sto:";
        char check_res[5]="res:";

        //While loop reads Serial input into input buffer.

        //Only continue if input buffer has 7 entries, otherwise ignore input buffer. 
        //All commands from RasPi shall come in a format of 7 characters "abc:xyz" 
        //      where 'abc' is an identifying string and 'xyz' is some data/information.
    //    if (i==7) {
            //Instantiate counters that will be used to verify known commands.
            int verified_pos=0;
            int verified_hea=0;
            int verified_dep=0;
            int verified_zer=0;
            int verified_set=0;
            int verified_tst=0;
            int verified_sto=0;
            int verified_res=0;
            //While loop checks first 4 characters of input buffer and attempts to match
            //      against known commands.
            for (int q=0; q<3; q++) {
                //Increment verified_pos if a match is found between Serial input buffer
                //      and Position command format.
                if (input[q]==check_pos[q]) {
                    verified_pos++;
                    pc.printf("verified for pos %c at %i\r\n",input[q],q);
                }
                //Increment verified_hea if a match is found between Serial input buffer
                //      and Heading command format.
                if (input[q]==check_hea[q]) {
                    pc.printf("verified for hea %c at %i\r\n",input[q],q);
                    verified_hea++;
                }
                if (input[q]==check_dep[q]) {
                    pc.printf("verified for dep %c at %i\r\n",input[q],q);
                    verified_dep++;
                }
                if (input[q]==check_zer[q]) {
                    pc.printf("verified for zer %c at %i\r\n",input[q],q);
                    verified_zer++;
                }
                if (input[q]==check_set[q]) {
                    pc.printf("verified for set %c at %i\r\n",input[q],q);
                    verified_set++;
                }
                if (input[q]==check_tst[q]) {
                    pc.printf("verified for tst %c at %i\r\n",input[q],q);
                    verified_tst++;
                }
                if (input[q]==check_sto[q]) {
                    pc.printf("verified for sto %c at %i\r\n",input[q],q);
                    verified_sto++;
                }
                if (input[q]==check_res[q]) {
                    pc.printf("verified for res %c at %i\r\n",input[q],q);
                    verified_res++;
                }
            }

            //If first 4 characters from Serial input buffer match Position command format,
            //      execute "switch_pos" function.
            if (verified_pos==3) {
                int change=(input[6]-'0');
                switch_pos(change);
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
                az_thruster_logic(hea);
                //stop port and stbd thrusters
                if (wait_call_threads_available()==1) call_threads(0);
            }
            
            //If first 4 characters from Serial input buffer match Depth command format,
            //      execute "depth" function.
            if (verified_dep==3) {
                //Correct for ascii '0', and reform 3digit decimal number
                int dep100=(input[4]-'0');
                int dep10=(input[5]-'0');
                int dep1=(input[6]-'0');
                int dep=(dep100*100)+(dep10*10)+(dep1*1);
                pc.printf("log: depth rx: %i\r\n",dep);
                el_thruster_logic(dep);
                //stop fwd and aft thrusters
                if (wait_call_threads_available()==1) call_threads(99);
            }
            if (verified_zer==3) {
                press_sensor.calculate();
    //            zero_depth=press_sensor.get_D1();
                zero_depth=press_sensor.pressure();
				pc.printf("log: zero depth set %f\r\n",zero_depth);
                zero_set=1;
				az_data();
            }
            if (verified_set==3) {
                press_sensor.calculate();
    //            cal_depth=press_sensor.get_D1();
                cal_depth=press_sensor.pressure();
				pc.printf("log: cal depth set %f\r\n",cal_depth);
                cal_set=1;
				az_data();
            }
            if (zero_set && cal_set) {
                //set ready_data bit 4 to indicate that depth has been set
                ready_data=(ready_data&0xfff7)|0x0008;
            }
            if (verified_tst==3) {
                int tst_mode=(input[4]-'0');
                int tst_data_dir=(input[5]-'0');
                int tst_data_move=(input[6]-'0');
                //For both tst_modes, tst_data_msb=0 stops all thrusters.
                //tst_mode 0 is for automated testing of 4 movements (tst_data_msb[0,4]) for fixed 3s.
                if (tst_mode==0) {
                    pc.printf("log: tst, mode: %i, dir: %i, move: %i\r\n",tst_mode,tst_data_dir,tst_data_move);
                    //Thread test;
                    //test.start(callback(test_direction, &tst_data_move));
                    test_direction(tst_data_move);
                }
                //tst_mode 1 is for manual commands of 4 movements for indefinite periods.
                if (tst_mode==1) manual_direction(tst_data_dir,tst_data_move);
                }
            if (verified_sto==3) {
				pc.printf("log: stop command received\r\n");
                //Stop az thrusters.
                if (wait_call_threads_available()==1) call_threads(0);
                //Stop el thrusters.
                if (wait_call_threads_available()==1) call_threads(99);
				pc.printf("log: stop command executed\r\n");
            }
            if (verified_res==3) {
				pc.printf("log: Reset mbed received. See you on the other side.\r\n");
                NVIC_SystemReset();
				pc.printf("log: Reset failed. The show goes on.\r\n");
            }
        command_available=1;
        input_thread.join();
		pc.printf("log: horizon thread joined\r\n");
    }
}

int main() {
    //engage plaidspeed
    pc.baud(115200);
//    imu.set_mounting_position(MT_P3);
    press_sensor.init(sensor_rate);
    press_sensor.density(997);

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

    //Look for serial input commands and send to 'commands' function.
    //If no serial input commands, stream data.
    while(1) {
        if (pc.readable()) commands();
        else az_data();
        wait_ms(10);
    }
}



