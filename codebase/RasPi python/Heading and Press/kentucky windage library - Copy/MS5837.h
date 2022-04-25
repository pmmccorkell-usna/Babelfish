/* Blue Robotics Arduino MS5837-30BA Pressure/Temperature Sensor Library
------------------------------------------------------------
 
Title: Blue Robotics Arduino MS5837-30BA Pressure/Temperature Sensor Library
Description: This library provides utilities to communicate with and to
read data from the Measurement Specialties MS5837-30BA pressure/temperature 
sensor.
Authors: Rustom Jehangir, Blue Robotics Inc.
         Adam Šimko, Blue Robotics Inc.
-------------------------------
The MIT License (MIT)
Copyright (c) 2015 Blue Robotics Inc.
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
-------------------------------*/ 
#include "mbed.h"

#ifndef MS5837_H
#define MS5837_H

#define ms5837_reset       0x1E // Sensor Reset


#define MS5837_RX_DEPTH 3 //
#define MS5837_TX_DEPTH 2 //

#define ms5837_addr_no_CS  0x76 //0b1110110
//#define MS5837_ADDRESS              0x76
//#define MS5837_ADDR_WRITE               0x7C
//#define MS5837_ADDR_READ                0x7D
#define ms5837_ADCread           0x00
#define ms5837_PROMread          0xA0
//Data sheet pg3 for resolution, pg2 for conversion time
#define MS5837_CONVERT_D1_256     0x40  //0.54ms wait, .110 mbar res -> 1.0 mm resolution
#define MS5837_CONVERT_D1_8192    0x4A  //20ms wait, .016 mbar res -> 0.2 mm resolution
//Data sheet pg3 for resolution, pg2 for conversion time
#define MS5837_CONVERT_D2_256      0x50  //0.54ms wait, .012 C res
#define MS5837_CONVERT_D2_8192    0x5A  //20ms wait, .002 C res


class MS5837 {
private:
    uint32_t D1, D2;
    uint16_t C[8];
    double PRESS, TEMP;
    char ms5837_rx_data[MS5837_RX_DEPTH];
    char ms5837_tx_data[MS5837_TX_DEPTH];
    char convD1;
    char convD2;
    float wait_time;
    float fluidDensity;
    uint8_t crc4(uint16_t n_prom[]);

public:
    MS5837 (PinName sda, PinName scl,
            char ms5837_addr = ms5837_addr_no_CS  )
            : i2c( sda, scl ), device_address( ms5837_addr << 1 ) {
    }
    static const float Pa;
    static const float bar;
    static const float mbar;

    void init(int rate);
    void prom(void);
    void reset(void);
    void set_rate(int set_rate);
    void density(float fluid_density);
    void convertD1(void);
    void convertD2(void);
    uint32_t read_adc(void);
    void calculate(void);
//    double pressure(float conversion = 1.0f);
	double pressure(void);
    double temperature();
    uint64_t get_D1(void);
    uint64_t get_D2(void);
	int64_t Cal1,Cal2,Cal3,Cal4,Cal5,Cal6;
    uint16_t cal_data(int array);
    double depth();


private:
    I2C  i2c;
    char device_address;
};

#endif



