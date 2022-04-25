//Bar02 Ultra High Resolution 10m Depth/Pressure Sensor sample
//MS5837-02BA onry
//ArduinoLibrary->mbedLibrary

//https://os.mbed.com/teams/POTLESS/code/MS5837/
//https://github.com/bluerobotics/BlueRobotics_MS5837_Library

#include "mbed.h"
#include "MS5837.h"


Serial pc(USBTX, USBRX,115200); // tx, rx

//I2C i2c(I2C_SDA,I2C_SCL);
MS5837 sensor(I2C_SDA, I2C_SCL, ms5837_addr_no_CS);


int main() {
  // Initialize pressure sensor
  // Returns true if initialization was successful
  // We can't continue with the rest of the program unless we can initialize the sensor
    printf("start"); 
    sensor.MS5837Reset();
    sensor.MS5837Init();
    printf("ok"); 

    
    while(1) {    
    
        sensor.Barometer_MS5837();

        printf("Pressure: "); 
        printf("%f",sensor.MS5837_Pressure()); 
        printf(" mbar");
          
        printf("Temperature: "); 
        printf("%f",sensor.MS5837_Temperature()); 
        printf(" deg C ");
          
        printf("Depth: "); 
        printf("%f",sensor.depth()); 
        printf(" m ");
          
        printf("Altitude: "); 
        printf("%f",sensor.altitude()); 
        printf(" m above mean sea level\r\n");
        
            
        wait(1);
    }
}