#include "mbed.h"
#include "MS5837.h"

/*const float MS5837::Pa = 100.0f;
const float MS5837::bar = 0.001f;
const float MS5837::mbar = 1.0f;
*/

void MS5837::init(int rate)
{
    reset();
    prom();
    //default to 8192 OverSampling Rate (OSR)
    set_rate(rate);
    //default to freshwater fluid density
    //1029 for seawater
    density(997);
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
void MS5837::prom(void)
{
    uint8_t i,j;
    for (i=0; i<8; i++) {
        j = i;
        ms5837_tx_data[0] = ms5837_PROMread + (j<<1);
        if ( i2c.write( device_address,  ms5837_tx_data, 1 ) );
        if ( i2c.read( device_address,  ms5837_rx_data, 2 ) );
        C[i]   = ms5837_rx_data[1] + (ms5837_rx_data[0]<<8);
    }
}

void MS5837::set_rate(int rate) {
        switch(rate) {
            case 256:
                convD1=0x40;
                convD2=0x50;
                wait_time=0.56;
            break;
            case 512:
                convD1=0x42;
                convD2=0x52;
                wait_time=1.1;
            break;
            case 1024:
                convD1=0x44;
                convD2=0x54;
                wait_time=2.17;
            break;
            case 2048:
                convD1=0x46;
                convD2=0x56;
                wait_time=4.32;
            break;
            case 4096:
                convD1=0x48;
                convD2=0x58;
                wait_time=8.61;
            break;
            default:
            case 8192:
                convD1=0x4A;
                convD2=0x5A;
                wait_time=17.2;
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
    printf("D1 addy: %c\r\n",ms5837_tx_data[0]);
    if ( i2c.write( device_address,  ms5837_tx_data, 1 ) );
    wait_ms(wait_time);
}

/* Start the sensor temperature conversion */
void MS5837::convertD2(void)
{
    ms5837_tx_data[0] = convD2;
    printf("D2 addy: %c\r\n",ms5837_tx_data[0]);
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

    convertD1();
    D1=read_adc();
//	printf("adc called D1: %x\r\n",D1);
    convertD2();
    D2=read_adc();
//	printf("adc called D2: %x\r\n",D2);
	printf("INSIDE CALC, D1: %i, D2: %i\r\n",D1,D2);

    int64_t dT = 0;
    int64_t SENS = 0;
    int64_t OFF = 0;
    int64_t SENSi = 0;
    int64_t OFFi = 0;  
    int64_t Ti = 0;    
    int64_t OFF2 = 0;
    int64_t SENS2 = 0;
    int64_t Cal1=0;
	int64_t Cal2=0;
	int64_t Cal3=0;
	int64_t Cal4=0;
	int64_t Cal5=0;
	int64_t Cal6=0;
    
	int32_t buffer;
	uint32_t un_buffer;
	un_buffer = (C[1] * 0x10000);	//2^16
	Cal1=un_buffer;
	buffer=C[2];
	printf("C2 buffer: %lli\r\n",buffer);
	Cal2=C[2]; //2^17
	Cal3=C[3];
	Cal4=C[4];
	Cal5=0x100*C[5];
	Cal5=(-1*Cal5);
	Cal6=C[6];
//	for (int j=0; i<17; i++) {
		
//	}


for (int i=1; i<7; i++) {
	printf("C[%i]: %x\r\n",i,C[i]);
}


printf("Cal 1: %lli\r\n",Cal1);
printf("Cal 2: %lli\r\n",Cal2);
printf("Cal 3: %lli\r\n",Cal3);
printf("Cal 4: %lli\r\n",Cal4);
printf("Cal 5: %lli\r\n",Cal5);
printf("Cal 6: %lli\r\n",Cal6);
//printf("Cal 1: %x, 2: %x, 3: %x\r\n",Cal1,Cal2,Cal3);
//printf("Cal 4: %x, 5: %x, 6: %x\r\n",Cal4,Cal5,Cal6);


//	buffer1=0x100*C[5];
//	dT=(-1*buffer1);
//	printf("C[5]: %i, buffer: %i, dT precalc: %i\r\n",C[5],buffer,dT);
    dT = (Cal5+D2);
	printf("dT postcalc: %lli\r\n",dT);
    
    //First order compensation.
    //See pg7 of datasheet for 1st order calculations.

    //[SENS]itivity = SENSt1 + (TCS * dT)
    //      = (C1*2^16)+(C3*dT)/2^7
//    SENS = (((Cal[1]))*65536LL)+(((int64_t(Cal[3]))*dT)/128);
	SENS=Cal1+((Cal3*dT)/128);
	printf("SENS: %i\r\n",SENS);

    //[OFF]set = Offt1 + (TCO * dT)
    //      = (C2*2^17)+(C4*dT)/2^6
//    OFF = (((Cal[2]))*131072LL)+(((int64_t(Cal[4]))*dT)/64);
	OFF=Cal2+((Cal4*dT)/64);
	printf("OFF: %i\r\n",OFF);
    
    //Temp compensated Pressure (10-1200mbar with .01mbar res)
    //[P]ressure = (D1*SENS) - OFF
    //          = [ D1 * (SENS/2^21) - OFF ] / (2^15)
    PRESS = ((D1*SENS/(2097152LL))-OFF)/(32768LL);

    //Actual Temperature [-40,85] deg C w/ 0.01 resolution
    //[TEMP]erature = 20deg + (dT*TEMPSENS)
    //          = 2000 + (dT * c6)/2^23
    TEMP = 2000+(((dT)*Cal6)/8388608LL);

    //Second order compensation.
    //See pg8 of data sheet for 2nd order calculations.

    //Low temp
    if((TEMP/100)<20){
        // Ti = (11*dt^2)/(2^35)
        Ti = (11*(dT)*(dT))/(34359738368LL);
        // Offi = 31 * (Temp - 2000)^2 / 2^3
        OFFi = (31*(TEMP-2000)*(TEMP-2000))/8;
        // SENSi = 63 * (TEMP - 2000)^2 / 2^5
        SENSi = (63*(TEMP-2000)*(TEMP-2000))/32;
    }


    OFF2 = (OFF-OFFi);           //Calculate pressure and temp second order
    SENS2 = (SENS-SENSi);
    
    TEMP = (TEMP-Ti);

    //[P]ressure = (D1*SENS) - OFF
    //      = (D1*SENS)/2^21 - (OFF/2^15)
    PRESS = ((((D1*SENS2)/2097152LL)-OFF2)/32768LL);
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

double MS5837::temperature() {
    return (TEMP/100.0f);
}

uint16_t MS5837::cal_data(int array) {
    return (C[array]);
}

uint64_t MS5837::get_D1(){
    return D1;
}

uint64_t MS5837::get_D2(){
    return D2;
}

//1. Get Pascals from pressure function
//2. Subtract P atmosphere (101.3 kPa)
//3. Calculate P fluid divisor: fluid density(kg/m3) * g(m/s2) -> kg*m/m3s2 -> N/m3 -> Pa/m
//4. ???
//5. (Pa / (Pa / m) = meters!
double MS5837::depth() {
    //range [-10.36,.... something isnt right
//    double depth_meters = (pressure(MS5837::Pa)-101300LL)/(fluidDensity*9.80665f);
//    double depth_cm = (depth_meters/100);
double offset_mbar=2.0621;
double max_mbar=2.0691;
double distance=10.16;
double coefficient=(max_mbar-offset_mbar)/distance;
double pressure_mbar=pressure(1);
double depth_cm=(pressure_mbar-offset_mbar)/coefficient;

    //because this is the 02BA depth sensor.
    return depth_cm;
}

uint8_t MS5837::crc4(uint16_t n_prom[]) {
    uint16_t n_rem = 0;

    n_prom[0] = ((n_prom[0]) & 0x0FFF);
    n_prom[7] = 0;

    for ( uint8_t i = 0 ; i < 16; i++ ) {
        if ( i%2 == 1 ) {
            n_rem ^= (uint16_t)((n_prom[i>>1]) & 0x00FF);
        } else {
            n_rem ^= (uint16_t)(n_prom[i>>1] >> 8);
        }
        for ( uint8_t n_bit = 8 ; n_bit > 0 ; n_bit-- ) {
            if ( n_rem & 0x8000 ) {
                n_rem = (n_rem << 1) ^ 0x3000;
            } else {
                n_rem = (n_rem << 1);
            }
        }
    }
    
    n_rem = ((n_rem >> 12) & 0x000F);

    return n_rem ^ 0x00;
}

