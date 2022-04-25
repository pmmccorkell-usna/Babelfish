// Uses attach_us to generate servo pulse on single output pin
// J. Bradshaw 20140925
/* Copyright (c) 2015, jbradshaw (http://mbed.org)
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
*/ 
#include "mbed.h"
#include "ServoOut.h"

ServoOut::ServoOut(PinName pin) : _pin(pin) {
    _pin = 0;           // Initialize output pin as off
    _sCycle = 0;        // initialize servo cycle as 0
    pulseMin=900;       // default minimum (except 0) servo pulse width
    pulseMax=2100;      // maximum servo pulse width
    pulse_us=0;         // Initialize servo pulse width as 0
    ticker = new Ticker();  // instantiate new Ticker object
    ticker->attach_us(this,&ServoOut::tickFunct,20000); //begin tickFunction in 20ms
}

void ServoOut::tickFunct(void){
    
    if(pulse_us == 0){  //first check to see if the pulse width is 0 (OFF, servo not powered)
        _pin=0;         // disable servo pulse output
        ticker->attach_us(this,&ServoOut::tickFunct,20000); // recall this function in 20ms
        return;
    }
    
    else{        
        if(pulse_us < pulseMin)     //keep within bounds
            pulse_us = pulseMin;
        if(pulse_us > pulseMax)
            pulse_us = pulseMax;
        
        if(_sCycle){   //mode == 1
            //pulse was high, now drop low for remaining time
            ticker->attach_us(this,&ServoOut::tickFunct,20000-pulse_us);; // recall this function in 20ms - servo pulse width (servo pulse low)
            _pin=0;    
            _sCycle=0;
        }
        else{   //mode == 0
            ticker->attach_us(this,&ServoOut::tickFunct,pulse_us);; // recall this function in 20ms - pulse width (servo pulse high)
            _pin=1;        
            _sCycle=1;
        }
    } //pulse != 0 (OFF)
}

void ServoOut::write(int pulse){
    pulse_us = pulse;    
}

int ServoOut::read(){
    return pulse_us;    
}

ServoOut& ServoOut::operator= (int pulse_us) { 
    write(pulse_us);
    return *this;
}
 
ServoOut& ServoOut::operator= (ServoOut& rhs) {
    write(rhs.read());
    return *this;
}
 
ServoOut::operator int() {
    return read();
}