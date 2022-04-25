#include "mbed.h"
#include "BNO055.h"

Serial pc(USBTX, USBRX);
I2C i2c(p9,p10);
DigitalOut pwr_on(p30);
BNO055 imu(i2c,p26);
//DigitalOut led1(LED1);

//instantiate global ready to 0
uint32_t ready = 0x00000000;

BNO055_ID_INF_TypeDef       bno055_id_inf;
BNO055_EULER_TypeDef        euler_angles;
//BNO055_QUATERNION_TypeDef   quaternion;
//BNO055_LIN_ACC_TypeDef      linear_acc;
BNO055_GRAVITY_TypeDef      gravity;
//BNO055_TEMPERATURE_TypeDef  chip_temp;

// Instantiate motor driver PWM pins, duty cycle (%), and period (us)
PwmOut Lfwd(p21); // left forward
PwmOut Lrev(p22); // left reverse
PwmOut Rfwd(p24); // right forward
PwmOut Rrev(p25); // right reverse
float dutycycle=0.40;
int period=60;

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

//Function to setup motor PWM for right turn
void TurnRight() {
    //mask off other bits and set Rmotor status to on
    ready=(ready & 0xfffffff7) + 0x8;
//    pc.printf("Turning Right\r\n");
    Rfwd.period_us(period);
    Rfwd.write(dutycycle);
    Lrev.period_us(period);
    Lrev.write(dutycycle);
}

//Function to setup motor PWM for left turn
void TurnLeft() {
    //mask off other bits and set Lmotor status to on
    ready=(ready & 0xffffffef) + 0x10;
//  pc.printf("Turning Left\r\n");
    Lfwd.period_us(period);
    Lfwd.write(dutycycle);
    Rrev.period_us(period);
    Rrev.write(dutycycle);
}

//Function to stop all motors
void Stop_motors() {
    //Set PW on all PWM pins to 0
    Lfwd.write(0);
    Lrev.write(0);
    Rfwd.write(0);
    Rrev.write(0);
    //zero out Lm and Rm status bits
    ready=(ready & 0xffffffe7);
}

//Function to handle motor logic
void motors(int desired_heading) {
	//Pull heading from BNO and correct for 16bit/deg resolution.
    float current_heading=((data(0x1234abcd))/0x10);
	//Set acceptable tolerance in deg, for heading accuracy.
    float tolerance = 0.2;
	
	//Error handling. Only continue for 0-360 range.
    if (desired_heading>=0 and desired_heading<=360) {
		//Set default direction (+ turn right, - turn left)
		
		//Calculate how far to turn in degrees.
        float diff = abs(desired_heading-(current_heading));
        //Correct for 360-0 edge cases if 'diff'erence is greater than 180.
        //Change direction and recalculate for accurate 'tolerance' comparison.
        if (diff>180) {
            if (desired_heading>180) diff=((current_heading+180)-(desired_heading-180));
            if (current_heading>180) diff=((desired_heading+180)-(current_heading-180));
        }
		
		
        while (diff > tolerance) {
	        int dir=0;
            current_heading=((data(0x1234abcd))/0x10);
            diff=abs((desired_heading)-(current_heading));
            if (diff>180) {
	            dir=1;
                if (desired_heading>180) diff=((current_heading+180)-(desired_heading-180));
                if (current_heading>180) diff=((desired_heading+180)-(current_heading-180));
			}
			if ((diff>tolerance) and (dir==0)) {
				if ((desired_heading-current_heading)>0) TurnRight();
				if ((desired_heading-current_heading)<0) TurnLeft();
//          	pc.printf("Current heading: %i\r\n",current_heading);
			}
			if ((diff>tolerance) and (dir==1)) {
				if ((desired_heading-current_heading)>0) TurnLeft();
				if ((desired_heading-current_heading)<0) TurnRight();
			}
            wait_ms(10);
        }
        Stop_motors();
//      pc.printf("heading: %i, diff: %i\r\n",current_heading,diff);
    }
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
    char input[1000];
    //check_pos char array used to filter Position commands.
    char check_pos[5]="pos:";
    //check_hea char array used to filter Heading commands.
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
            motors(hea);
        }
    }
}

int main() {
    //engage plaidspeed
    pc.baud(115200);
//    imu.set_mounting_position(MT_P3);

    //If not ready, reset BNO.
    while (imu.chip_ready() == 0) {
        do {
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

