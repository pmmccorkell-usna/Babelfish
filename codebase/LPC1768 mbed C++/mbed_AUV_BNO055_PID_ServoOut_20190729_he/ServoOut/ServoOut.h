// J. Bradshaw 20150409
/** ServoGen Class for Generating servo pulses on single output pin
 * Copyright (c) 2014, jbradshaw (http://mbed.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
*
* Example:
* @code
*#include "mbed.h"
*#include "ServoOut.h" 
* 
*ServoOut servo1(p21);
*ServoOut servo2(p24);
*DigitalOut led1(LED1);
* 
*int main() {
*    servo1.pulse_us = 0;        
*    // spin in a main loop. flipper will interrupt it to call flip
*    while(1) {
*        for(float cycle=0.0;cycle<2.0*3.14159;cycle+=.003){
*            //small amplitude sine wave on servo1
*            servo1.pulse_us = 300 * sin(cycle) + 1500;            
*            //ramp up the second servo2 channel relative to cycle
*            servo2.pulse_us = 1000 + cycle*159; // .001/(2*PI)
*            
*            wait(.001); //short delay
*        }
*        led1 = !led1;   //toggle led1 to indicate activity
*    }//while(1)
*}//main
*
*@endcode
*/

#ifndef MBED_SERVOOUT_H
#define MBED_SERVOOUT_H
 
#include "mbed.h"

/**
 * ServoOut Class.
 */

class ServoOut {
public:
    /**
     * Constructor.
     *
     * @param pin - servo pulse output pin
     */         
     ServoOut(PinName pin);
    
    int pulse_us;       //determines the pulse width in microseconds (default is 0 = OFF)
    int pulseMin;       //minimum pulse width (default is 900 microseconds)
    int pulseMax;       //maximum pulse width (default is 2100 microseconds)
    
    void write(int pulse);  //
    int read(void);
    
        /**  Shorthand for the write and read functions */
    ServoOut& operator= (int pulse_us);
    ServoOut& operator= (ServoOut& rhs);
    operator int();
  
private:
    void tickFunct(void);   //Function that takes care of the servo pulse generation
    Ticker *ticker;         //pointer to ticker object
    DigitalOut _pin;        //make assigned pin a digital output
    int _sCycle;            //flag to toggle for keeping track of servo cycle
};
 
#endif