/*
BM017_Arduino_color_sensing:  This program interfaces to the AMS TCS34725 color light
to digital converter IC.  It uses the Arduino I2C interface.  

Schematics associated with the BM017 module may be used for hardware wiring information.
See the user datasheet at www.solutions-cubed.com for additional information.

*/
 
#include "mraa.hpp"
#include <cassert>
#include <csignal>
#include <iostream>


const int ledPin = 8;  // used to drive the white illumination LED

#define SensorAddress 0x29 //
#define EnableAddress 0xa0 // register address + command bits
#define ATimeAddress 0xa1 // register address + command bits
#define ControlAddress 0xaf // register address + command bits
#define IDAddress 0xb2 // register address + command bits
#define ColorAddress 0xb4 // register address + command bits

int running = 1;
#define MS 1000

/*  
Load the i2cWriteBuffer with the data you want to write and 
then call this routine with appropriate address and command 
bits and the number of bytes you are writing.
*/

uint8_t registers[2]; 

void sig_handler(int signo) {
    if (signo == SIGINT) {
        printf("closing spi nicely\n");
        running = 0;
    }
}

/*  
Send register address to this function the
number of bytes you want to read and it 
returns the byte value for the register's 
contents in the i2cReadBuffer
*/
byte Readi2cRegisters(int numberbytes, byte command)
{
  byte i = 0;
   
    Wire.beginTransmission(SensorAddress);   // Write address of read to sensor
    Wire.write(command);
    Wire.endTransmission();
  
    delayMicroseconds(100);      // allow some time for bus to settle      

    Wire.requestFrom(SensorAddress,numberbytes);   // read data
    for(i=0;i<numberbytes;i++)
      i2cReadBuffer[i] = Wire.read();
    Wire.endTransmission();   

    delayMicroseconds(100);      // allow some time for bus to settle      
}  

void init_TCS34725(void)
{
  i2cWriteBuffer[0] = 0xf6;
  Writei2cRegisters(1,ATimeAddress);    // RGBC timing is 256 - contents x 2.4mS =  
  i2cWriteBuffer[0] = 0x00;
  Writei2cRegisters(1,ControlAddress);  // 1X gain RGBC gain control
  i2cWriteBuffer[0] = 0x03;
  Writei2cRegisters(1,EnableAddress);    // enable ADs and oscillator for sensor  
}

void get_TCS34725ID(void)
{
  Readi2cRegisters(1,IDAddress);
  if (i2cReadBuffer[0] == 0x44)            // needs double ==
    Serial.println("TCS34725 is present");    
  else
    Serial.println("TCS34725 not responding");    
}

/*
Reads the register values for clear, red, green, and blue.
*/
void get_Colors(void)
{
  unsigned int clear_color = 0;
  unsigned int red_color = 0;
  unsigned int green_color = 0;
  unsigned int blue_color = 0;
  
  Readi2cRegisters(8,ColorAddress);
  clear_color = (unsigned int)(i2cReadBuffer[1]<<8) + (unsigned int)i2cReadBuffer[0];
  red_color = (unsigned int)(i2cReadBuffer[3]<<8) + (unsigned int)i2cReadBuffer[2];
  green_color = (unsigned int)(i2cReadBuffer[5]<<8) + (unsigned int)i2cReadBuffer[4];
  blue_color = (unsigned int)(i2cReadBuffer[7]<<8) + (unsigned int)i2cReadBuffer[6];

  // send register values to the serial monitor 

  printf("clear color=");
  printf(clear_color, DEC);    
  printf(" red color=");
  printf(red_color, DEC);    
  printf(" green color=");
  printf(green_color, DEC);    
  printf(" blue color=");
  printf(blue_color, DEC);


 // Basic RGB color differentiation can be accomplished by comparing the values and the largest reading will be 
 // the prominent color

  if(clear_color > 150)
    printf("No Object Detected");
  else if((red_color>blue_color) && (red_color>green_color))
    printf("detecting red");
  else if((green_color>blue_color) && (green_color>red_color))
    printf("detecting green");
  else if((blue_color>red_color) && (blue_color>green_color))
    printf("detecting blue");
  else
    printf("color not detectable");
  
  
  // Space
  printf(""); 
}  

int main() {
    // Handle Ctrl-C quit
    signal(SIGINT, sig_handler);

    // Edison i2c bus is 6
    mraa::I2c *i2c = new mraa::I2c(6);
    assert(i2c != NULL);

    initPWM(i2c);

    while (running) {
        // Alternate two locations with 2-sec delay
        setServoPosition(i2c, 0, 0.2);
        sleep(2.0);
        setServoPosition(i2c, 0, 0.8);
        sleep(2.0);
    }
}

