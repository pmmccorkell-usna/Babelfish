#include "mbed.h"
Serial pc(USBTX, USBRX);
int baudrate = 115200;

SPISlave piSPI(p5,p6,p7,p8);    //mosi,miso,sclk,cs


int main() {
    pc.baud(baudrate);
    uint8_t int8 = 0x0;
    while(1) {
        uint8_t response = 0x0;
        if (piSPI.receive()) {
            int8=piSPI.read();
            if (int8) {
                response=int8-1;
            }
            piSPI.reply(response);
            pc.printf("\r\n");
            pc.printf("int8: %x",int8);
            pc.printf("\r\n");
        }
    }
}
