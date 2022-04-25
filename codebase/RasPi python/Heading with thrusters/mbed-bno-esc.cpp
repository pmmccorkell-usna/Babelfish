#include "mbed.h"
#include "BNO055.h"

//Setup USB Serial
Serial pc(USBTX, USBRX);

//Setup BNO055 over I2C
I2C i2c(p9,p10);
DigitalOut pwr_on(p30);
BNO055 imu(i2c,p8);

//DigitalOut led1(LED1);

//instantiate global ready to 0
uint32_t ready = 0x00000000;

BNO055_ID_INF_TypeDef       bno055_id_inf;
BNO055_EULER_TypeDef        euler_angles;
//BNO055_QUATERNION_TypeDef   quaternion;
//BNO055_LIN_ACC_TypeDef      linear_acc;
BNO055_GRAVITY_TypeDef      gravity;
//BNO055_TEMPERATURE_TypeDef  chip_temp;

// ESC specs data
double esc_freq=400;     //Standard servo rate -> 400Hz
double base_pw=1.5;     //ms
double null_pw=0.0;

double esc_period=(1000/esc_freq);  //ms
double esc_step=(30/1000);  //25us per data sheet Blue Robotics BESC-R1
//double esc_sample_freq=400;    //400Hz update frequency per data sheet
//double esc_time_ms=(1000/esc_sample_freq);
double esc_range_spec=.4;   // 1.1-1.9ms pw per data sheet
double esc_range=esc_range_spec*0.9;    //only use x% of full range
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

class Thruster {
    public:
        Thruster(PinName pin, float dir);
        void set_period(double thruster_time);
        void set_pw(double thruster_pw);
        double get_pw();
        void set_speed(double pntr);
    protected:
        PwmOut _pwm;
        float _d;
};

Thruster::Thruster(PinName pin, float dir) : _pwm(pin), _d(dir) {
    set_period(2.5);
    set_pw(1.5);
    //pc.printf("Thruster: %f\r\n",d);
}

void Thruster::set_period(double thruster_time) {
    double period=(thruster_time/1000);
    _pwm.period(period);
}

void Thruster::set_pw(double thruster_pw) {
    double s_pw=(thruster_pw/1000);
//  pc.printf("set_pw: %f\r\n",s_pw);
    _pwm.pulsewidth(s_pw);
}

double Thruster::get_pw() {
    double g_pw = (_pwm.read()*2.5);
    return g_pw;
}

void Thruster::set_speed(double speed_pw) {
    pc.printf("set_speed called: %f\r\n", speed_pw);
    pc.printf("speed dir: %f\r\n",_d);
    double target_pw=(_d*speed_pw)+base_pw;
    double current_pw=get_pw();
    double diff_pw=abs(target_pw-current_pw);
    pc.printf("target: %f, current: %f, difference: %f\r\n",target_pw,current_pw,diff_pw);
    //Time we want to take to get from 1.5 to [1.9 or 1.1]
//  double Delay=0.5; //500ms
    //Total # of steps / resolution of ESC
//  double Steps=((.0019-.0015)/esc_step);
    //Center of sigmoid curve
    double Center=.25; //in seconds, 50% ramp-up at center time
    //slope at Center
    double Slope=50; //rate of ramp up
    double time_count=0; //total time in while loop
    double dwell_time=0.5; //time to wait between while loop
    double tolerance_pw=.03; //30us
    double dir=0;
    while(diff_pw>tolerance_pw) {
        current_pw=get_pw();
        diff_pw=abs(target_pw-current_pw);
//      pc.printf("diff_pw: %f\r\n",diff_pw);
        if ((target_pw-current_pw)>=0) {
            dir=(1);
            //moving fwd
//            ready=(ready & 0xfffffff7) + 0x8;
        }
        if ((target_pw-current_pw)<0) {
            dir=(-1);
            //moving rev
//            ready=(ready & 0xffffffef) + 0x10;
        }
        double factor=((diff_pw)*(Sigmoid(1,Slope,Center,time_count)));
		if (factor<=0.03) factor=0.03;
        //double new_pw=base_pw+(diff_pw*factor);
		pc.printf("current: %f\r\n",current_pw);
        pc.printf("factor: %f, dir: %f, diff_pw: %f\r\n",factor,dir,diff_pw);
        double new_pw=(current_pw+(dir*factor));
        set_pw(new_pw);
        wait_ms(dwell_time);
        time_count+=dwell_time;
    }
    pc.printf("set_pw: %f\r\n",target_pw);
    set_pw(target_pw);
}

// Instantiate thrusters
Thruster port_thrust(p21,-1);
Thruster starboard_thrust(p22,1);
Thruster steadystate(p23,1); //just to keep ESC from beeping
Thruster fore_thrust(p24,-1);
Thruster aft_thrust(p25,1);

//Data function pulls data from BNO and sends over serial
//Accepts unsigned 32bit key word (k) that RasPi will use as trigger
uint16_t data(uint32_t k) {
    //Instantiate status array of 6 32-bit words.
    //First 16 bits of each 32-bit word are Identifiers for RasPi to correctly assign the trailing 16 bits of data.
    uint32_t status[6]={0};

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

    //For loop iterates through Status array to transmit 6 32-bit words over Serial with "\n" appended for Python in RasPi.
    int i;
    int l=(sizeof(status)/sizeof(uint32_t))-1;
    for (i=0; i<=l; i++) {
        pc.printf("%x\n", status[i]);
  }
 
  //return heading
  return h;
}

//Functions for each motor for threading. 
//Thread calls cannot accept more than 1 argument.
//Thread calls cannot have object.method and arguments.
//Thereby, these functions are necessary: 
// 1. Start new Thread
// 2. Call the Thruster object.method(arg) within this thread
// 3. ???
void Stbd_thread(double *d_pntr) {
    pc.printf("Stbd thread called\r\n");
    double pw=*d_pntr;
    starboard_thrust.set_speed(pw);
}
void Port_thread(double *d_pntr) {
    pc.printf("Port thread called\r\n");
    double pw=*d_pntr;
    port_thrust.set_speed(pw);
}
void Fore_thread(double *d_pntr) {
    pc.printf("Fore thread called\r\n");
    double pw=*d_pntr;
    fore_thrust.set_speed(pw);
}
void Aft_thread(double *d_pntr) {
    pc.printf("Aft thread called\r\n");
    double pw=*d_pntr;
    aft_thrust.set_speed(pw);
}

//Function to create new threads for motor logic
int call_threads(int select, double dir, double speed) {
	int return_val=0;
	double rev_speed=(-1*speed);
	Thread thruster1;
	Thread thruster2;
	switch (select) {
		//case 1, forwards or backwards
		case 1:
			ready=(ready & 0xffffffe7) + 0x18;
			if (dir==1) {
				pc.printf("Forward\r\n");
				thruster1.start(callback(Stbd_thread, &speed));
				thruster2.start(callback(Port_thread, &speed));
			}
			if (dir==-1) {
				pc.printf("Reverse\r\n");
				thruster1.start(callback(Stbd_thread, &rev_speed));
				thruster2.start(callback(Port_thread, &rev_speed));
			}
			break;

		//case 2, turn left or right
		case 2:
			if (dir==1) {
				pc.printf("Turn to Stbd\r\n");
				ready=(ready & 0xfffffff7) + 0x8;
				thruster1.start(callback(Stbd_thread, &speed));
				thruster2.start(callback(Port_thread, &rev_speed));
			}
			if (dir==-1) {
				pc.printf("Turn to Port\r\n");
				ready=(ready & 0xffffffef) + 0x10;
				thruster1.start(callback(Stbd_thread, &rev_speed));
				thruster2.start(callback(Port_thread, &speed));
			}
			break;

		//case 3, Up and Down
		case 3:
			if (dir==1) {
				pc.printf("Going up\r\n");
				thruster1.start(callback(Fore_thread, &speed));
				thruster2.start(callback(Aft_thread, &speed));
			}
			if (dir==-1) {
				pc.printf("Going down\r\n");
				thruster1.start(callback(Fore_thread, &rev_speed));
				thruster2.start(callback(Aft_thread, &rev_speed));
			}
			break;

		//case 4, pitch up/down
		case 4:
			if (dir==1) {
				pc.printf("Pitch Up\r\n");
//				ready=(ready & 0xfffffff7) + 0x8;
				thruster1.start(callback(Fore_thread, &speed));
				thruster2.start(callback(Aft_thread, &rev_speed));
			}
			if (dir==-1) {
				pc.printf("Pitch Down\r\n");
//				ready=(ready & 0xffffffef) + 0x10;
				thruster1.start(callback(Fore_thread, &rev_speed));
				thruster2.start(callback(Aft_thread, &speed));
			}
			break;
			
		//cases 5 and 6 reserved for roll should we ever get more motors

		//case 7, Emergency Surface
		case 7:
			pc.printf("Emergency Surface\r\n");
			thruster1.start(callback(Fore_thread, &esc_range_spec));
			thruster2.start(callback(Aft_thread, &esc_range_spec));
			break;
			
		//case 0, stop az thrusters
		default:
		case 0:
			pc.printf("Stop az thrusters\r\n");
			ready=(ready & 0xffffffef);
			thruster1.start(callback(Stbd_thread, &null_pw));
			thruster2.start(callback(Port_thread, &null_pw));
			break;
	}
	thruster1.join();
	thruster2.join();
	return_val=1;
	return return_val;
}

//Function to handle motor logic
void az_thruster_logic(int desired_heading) {
    //Pull heading from BNO and correct for 16bit/deg resolution.
    double current_heading=((data(0x1234abcd))/0x10);
    //Set acceptable tolerance in deg, for heading accuracy.
    double tolerance = 1.0;
    //The natural #
    pc.printf("Thruster Logic called\r\n");
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
            current_heading=((data(0x1234abcd))/0x10);
            diff=abs((desired_heading)-(current_heading));
            //Sigmoid data
            double center=20; //deg
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
			int check_threads=0;
			check_threads=call_threads(2,dir,speed);
			//Ensure existing threads are closed before restarting while loop and opening new threads.
			int i;
			for (i=0; check_threads!=1; i++) {
				wait_ms(0.001);
			}
//			ready=((ready & 0xffff001f)+(thread_count*0x20));
			if (check_threads==1) pc.printf("Threads rejoined, heading: %f, count: %i\r\n",current_heading, i);
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
    if (position>=0 and position<8) {
        switch (position) {
            case 1:
                imu.set_mounting_position(MT_P1);
                ready=((ready & 0xfffffff8)+0x1);
                break;
            case 2:
                imu.set_mounting_position(MT_P2);
                ready=((ready & 0xfffffff8)+0x2);
                break;
            case 3:
                imu.set_mounting_position(MT_P3);
                ready=((ready & 0xfffffff8)+0x3);
                break;
            case 4:
                imu.set_mounting_position(MT_P4);
                ready=((ready & 0xfffffff8)+0x4);
                break;
            case 5:
                imu.set_mounting_position(MT_P5);
                ready=((ready & 0xfffffff8)+0x5);
                break;
            case 6:
                imu.set_mounting_position(MT_P6);
                ready=((ready & 0xfffffff8)+0x6);
                break;
            case 7:
                imu.set_mounting_position(MT_P7);
                ready=((ready & 0xfffffff8)+0x7);
                break;
            case 0:
            default:
                imu.set_mounting_position(MT_P0);
                ready=((ready & 0xfffffff8));
                break;
        }
    }
}

//Commands function handles Serial input, checks for correct syntax, and calls appropriate function(s) to execute commands.
void commands() {
    //Instantiate input char array to buffer Serial input.
    char input[100];
    //check_pos char array used to filter Position commands.
    char check_pos[5]="pos:";
    //check_hea char array used to filter Go to Heading commands.
    char check_hea[5]="hea:";

    //While loop reads Serial input into input buffer.
    int i=0;
    while (pc.readable()) {
        input[i]=pc.getc();
//        pc.printf("read char %c to %i\r\n",input[i], i);
        i++;
    }

    //Only continue if input buffer has 7 entries, otherwise ignore input buffer. 
    //All commands from RasPi shall come in a format of 7 characters "abc:xyz" 
    //      where 'abc' is an identifying string and 'xyz' is a decimal number.
    if (i==7) {
//        pc.printf("checking 0-%c, 1-%c, 2-%c, 3-%c, 4-%c \r\n",check_pos[0],check_pos[1],check_pos[2],check_pos[3],check_pos[4]);

        //Instantiate counters that will be used to verify known commands.
        int verified_pos=0;
        int verified_hea=0;
        
        //While loop checks first 4 characters of input buffer and attempts to match
        //      against known commands.
        int q=0;
        while (q<4) {
            //Increment verified_pos if a match is found between Serial input buffer
            //      and Position command format.
            if (input[q]==check_pos[q]) {
                verified_pos++;
//                pc.printf("verified for pos %c at %i\r\n",input[q],q);
            }
            
            //Increment verified_hea if a match is found between Serial input buffer
            //      and Heading command format.
            if (input[q]==check_hea[q]) {
//                pc.printf("verified for hea %c at %i\r\n",input[q],q);
                verified_hea++;
            }
            /*           
            if (input[q]==check_mnt[q]) {
                verified_mnt++;
            }
            */
            q++;
        }
        
        //If first 4 characters from Serial input buffer match Position command format,
        //      execute "switch_pos" function.
        if (verified_pos==4) {
            int change=(input[6]-'0');
            switch_pos(change);
        }
        
        //If first 4 characters from Serial input buffer match Heading command format,
        //      execute "motors" function.
        if (verified_hea==4) {
            //Correct for ascii '0', and reform 3digit decimal number
            int hea100=(input[4]-'0');
            int hea10=(input[5]-'0');
            int hea1=(input[6]-'0');
            int hea=(hea100*100)+(hea10*10)+(hea1*1);
//            pc.printf("heading: %i\r\n",hea);
            az_thruster_logic(hea);
            //zero out ready status for left/right turn
            ready=(ready & 0xffffffe7);
			//stop port and stbd thrusters
			call_threads(0,1,null_pw);
        }
    }
}

int main() {
    //engage plaidspeed
    pc.baud(115200);
//    imu.set_mounting_position(MT_P3);
//  Stop_thrusters();

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
        ready=0xffff0000;
    }
//    uint32_t keyver=intake(ready);

    //Look for serial input commands and send to 'commands' function.
    //If no serial input commands, stream data.
    while(1) {
        if (pc.readable()) commands();
        else data(0x1234abcd);
        wait_ms(20);
    }
}

