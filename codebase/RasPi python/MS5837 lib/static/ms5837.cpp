#include "mbed.h"
#include "MS5837.h"

const float MS5837::Pa = 100.0f;
const float MS5837::bar = 0.001f;
const float MS5837::mbar = 1.0f;

//type 0 BAR02
//type 1 BAR30
void MS5837::init(int rate, int type)
{

    //default to 8192 OverSampling Rate (OSR)
    set_rate(rate);
    _type=type;
    _count=0;
    _atm=101300;
    //default to freshwater fluid density
    //1029 for seawater
    density(997);
    reset();
    prom();
    return;
}

/* Send soft reset to the sensor */
void MS5837::reset(void)
{
    /* transmit out 1 byte reset command */
    ms5837_tx_data[0] = ms5837_reset;
    if ( i2c.write( device_address,  ms5837_tx_data, 1 ) );
    printf("send soft reset\n");
    wait_ms(20);
}

/* read the sensor calibration data from rom */
void MS5837::prom(void) {
    uint8_t i,j;
    for (i=0; i<8; i++) {
        j = i;
        ms5837_tx_data[0] = ms5837_PROMread + (j<<1);
        if ( i2c.write( device_address,  ms5837_tx_data, 1 ) );
        if ( i2c.read( device_address,  ms5837_rx_data, 2 ) );
        C[i]   = ms5837_rx_data[1] + (ms5837_rx_data[0]<<8);
    }
    if (_type==0) {
        Cal1=((int64_t)C[1] * (1<<16)); //02BA
//        Cal1=((int64_t)C[1] * (1<<15));	//30BA
        Cal2=((int64_t)C[2]*(1<<17));	//02BA
//        Cal2=((int64_t)C[2]*(1<<16));	//30BA
        Cal3=C[3];
        Cal4=C[4];
        Cal5=0x100*C[5];				
		Cal5=((int64_t)C[5]*(1<<8));	
        Cal5=(-1*Cal5);
        Cal6=C[6];
    }
    else {
        Cal1=((int64_t)C[1] * (1<<15));
        Cal2=((int64_t)C[2] * (1<<16));
        Cal3=C[3];
        Cal4=C[4];
        Cal5=0x100*C[5];
        Cal5=(-1*Cal5);
        Cal6=C[6];
    }
}

/*Change Oversampling Rate. Significantly affects wait times to refresh data.*/
void MS5837::set_rate(int rate) {
        switch(rate) {
            case 256:
                convD1=0x40;
                convD2=0x50;
                //wait_time=0.56;
				wait_time=0.6;
            break;
            case 512:
                convD1=0x42;
                convD2=0x52;
                //wait_time=1.1;
				wait_time=1.17;
            break;
            case 1024:
                convD1=0x44;
                convD2=0x54;
                //wait_time=2.17;
				wait_time=2.28;
            break;
            case 2048:
                convD1=0x46;
                convD2=0x56;
                //wait_time=4.32;
				wait_time=4.54;
            break;
            case 4096:
                convD1=0x48;
                convD2=0x58;
                //wait_time=8.61;
				wait_time=9.04;
            break;
            default:
            case 8192:
                convD1=0x4A;
                convD2=0x5A;
                //wait_time=17.2;
				wait_time=18.08;
            break;
        }
        //add 10% to max listed wait time
        wait_time=(wait_time*1.1);
}

void MS5837::density(float fluid_density) {
    fluidDensity = fluid_density;
}

/* Start the sensor pressure conversion */
void MS5837::convertD1(void)
{
    ms5837_tx_data[0] = convD1;
//    printf("D1 addy: %c\r\n",ms5837_tx_data[0]);
    if ( i2c.write( device_address,  ms5837_tx_data, 1 ) );
    wait_ms(wait_time);
}

/* Start the sensor temperature conversion */
void MS5837::convertD2(void)
{
    ms5837_tx_data[0] = convD2;
//    printf("D2 addy: %c\r\n",ms5837_tx_data[0]);
    if ( i2c.write( device_address,  ms5837_tx_data, 1 ) );
    wait_ms(wait_time);
}

/* Read the previous started conversion results */
uint32_t MS5837::read_adc(void)
{
    uint32_t adc=0x0;
    wait_ms(wait_time);
    ms5837_tx_data[0] = ms5837_ADCread;
    if ( i2c.write( device_address,  ms5837_tx_data, 1 ) );
    if ( i2c.read( device_address,  ms5837_rx_data, 3 ) );
    adc = ms5837_rx_data[2] + (ms5837_rx_data[1]<<8) + (ms5837_rx_data[0]<<16);
//    printf("ADC value: %x \n\r", adc);
    return (adc);
}

void MS5837::calculate() {
    // Given C1-C6 and D1, D2, calculated TEMP and P
    // Do conversion first and then second order temp compensation
	//printf("C1: %x, C2: %x, C3: %x, C4: %x, C5: %x, C6: %x\r\n",C[1],C[2],C[3],C[4],C[5],C[6]);
	//printf("Cal1: %lli, Cal2: %lli, Cal3: %lli, Cal4: %lli, Cal5: %lli, Cal6: %lli\r\n",Cal1,Cal2,Cal3,Cal4,Cal5,Cal6);
    convertD1();
    D1=read_adc();
//  printf("adc called D1: %x\r\n",D1);

    //only update temp once every 10 cycles.
    if (_count==0) {
        convertD2();
        D2=read_adc();
        _count++;
    }
    else if (_count>19) _count=0;
    else _count++;
//  printf("adc called D2: %x\r\n",D2);
//  printf("INSIDE CALC, D1: %i, D2: %i\r\n",D1,D2);

    int64_t dT = 0;
    int64_t SENS = 0;
    int64_t OFF = 0;
    int64_t SENSi = 0;
    int64_t OFFi = 0;  
    int64_t Ti = 0;    
    int64_t OFF2 = 0;
    int64_t SENS2 = 0;
    
//  printf("C2: %i, Cal2: %lli\r\n",C[2],Cal2);
//  printf("C4: %i, Cal4: %lli\r\n",C[4],Cal4);


    dT = (Cal5+D2);
//  printf("dT: %lli\r\n",dT);

    //Actual Temperature [-40,85] deg C w/ 0.01 resolution
    //[TEMP]erature = 20deg + (dT*TEMPSENS)
    //          = 2000 + (dT * c6)/2^23
    TEMP = 2000+(((dT)*Cal6)/8388608LL);
//  printf("temp1: %f\r\n",TEMP);


    //First order compensation.
    //See pg7 of datasheet for 1st order calculations.

    //[OFF]set = Offt1 + (TCO * dT)
    //      = (C2*2^17)+(C4*dT)/2^6
    if (_type==0) OFF=Cal2+((Cal4*dT)/64);
    else OFF=Cal2+((Cal4*dT)/128);
//  printf("OFF: %lli\r\n",OFF);

    //[SENS]itivity = SENSt1 + (TCS * dT)
    //      = (C1*2^16)+(C3*dT)/2^7
    if (_type==0) SENS=Cal1+((Cal3*dT)/128);
    else SENS=Cal1+((Cal3*dT)/256);
//  printf("SENS: %lli\r\n",SENS);


    //Temp compensated Pressure (10-1200mbar with .01mbar res)
    //[P]ressure = (D1*SENS) - OFF
    //          = [ D1 * (SENS/2^21) - OFF ] / (2^15)
	if (_type==0) PRESS = ((D1*SENS/(2097152LL))-OFF)/(32768LL);
	else PRESS = ((D1*SENS/(2097152LL))-OFF)/(8192LL);
//  printf("pressure1: %f\r\n",PRESS);


    //Second order compensation.
    //See pg8 of data sheet for 2nd order calculations.

    //Low temp
    if((_type==0) and ((TEMP/100)<20)){
        // Ti = (11*dt^2)/(2^35)
        Ti = (11*(dT)*(dT))/(34359738368LL);
        // Offi = 31 * (Temp - 2000)^2 / 2^3
        OFFi = (31*(TEMP-2000)*(TEMP-2000))/8;
        // SENSi = 63 * (TEMP - 2000)^2 / 2^5
        SENSi = (63*(TEMP-2000)*(TEMP-2000))/32;
    }
	
	if (_type!=0) {
		if ((TEMP/100)<20) {
        Ti = ((3*(dT)*(dT))/(8589934592LL));
        OFFi = ((3*(TEMP-2000)*(TEMP-2000))/2);
        SENSi = ((5*(TEMP-2000)*(TEMP-2000))/8);
		}
		else {
			Ti=((2*(dT)*(dT))/(137438953472LL));
			OFFi=((1*(TEMP-2000)*(TEMP-2000))/16);
			SENSi=0;
		}
	}
	
    OFF2 = (OFF-OFFi);           //Calculate pressure and temp second order
    SENS2 = (SENS-SENSi);
    
    if (_type==0) TEMP = (TEMP-Ti);
	else TEMP = ((TEMP-Ti)/100);
	

    //[P]ressure = (D1*SENS) - OFF
    //      = (D1*SENS)/2^21 - (OFF/2^15)
	//divide by 100 in pressure function
    if (_type==0) PRESS = ((((D1*SENS2)/2097152LL)-OFF2)/32768LL);
	else PRESS=10*((((D1*SENS2)/2097152LL)-OFF2)/8192LL);

//	printf("Press: %f, Temp: %f\r\n",PRESS,TEMP);

/*
double division=D1;
    for (int i=0;i<15;i++) {
        division=(division/2);
    }
    PRESS=division;
*/
}


// "conversion" determines units
double MS5837::pressure(float conversion) {
    return ((PRESS*conversion)/100);
}

/*
double MS5837::pressure() {
    return PRESS;
}
*/

double MS5837::temperature() {
    return (TEMP/100.0f);
}

uint16_t MS5837::cal_data(int array) {
    return (C[array]);
}

int64_t MS5837::cal_post(int array) {
    int64_t returnval=0;
    switch (array) {
        case 1:
            returnval=Cal1;
            break;
        case 2:
            returnval=Cal2;
            break;
        case 3:
            returnval=Cal3;
            break;
        case 4:
            returnval=Cal4;
            break;
        case 5:
            returnval=Cal5;
            break;
        default:
        case 6:
            returnval=Cal6;
            break;
    }
    return returnval;
}

uint64_t MS5837::get_D1(){
    return D1;
}

uint64_t MS5837::get_D2(){
    return D2;
}

double MS5837::set_atm(){
    _count=0;
    calculate();
    _atm=pressure(Pa);
    return _atm;
}

//1. Get Pascals from pressure function
//2. Subtract P atmosphere (101300 Pa)
//3. Calculate P fluid divisor: fluid density(kg/m3) * g(m/s2) -> kg*m/m3s2 -> N/m3 -> Pa/m
//4. ???
//5. (Pa / (Pa / m) = meters!
double MS5837::depth() {

    double press=pressure(Pa);
    double depth_meters = (press-_atm)/(fluidDensity*9.80665f);
    double depth_cm = (depth_meters*100);

/*
double offset_mbar=5463;
double max_mbar=5515;
double distance=10.55;
double coefficient=(max_mbar-offset_mbar)/distance;
double pressure_mbar=PRESS;
double depth_cm=(pressure_mbar-offset_mbar)/coefficient;
*/
    return depth_cm;
}
