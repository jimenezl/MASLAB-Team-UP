// Compile with:
// g++ test_adafruit_color_sensor_real.cpp -o test_adafruit_color_sensor_real -lmraa
// Controls carousel depending on color of block.

#include "mraa.hpp"
#include <cassert>
#include <csignal>
#include <iostream>


#define SensorAddress 0x29 //
#define EnableAddress 0xa0 // register address + command bits
#define ATimeAddress 0xa1 // register address + command bits
#define ControlAddress 0xaf // register address + command bits
#define IDAddress 0xb2 // register address + command bits
#define ColorAddress 0xb4 // register address + command bits

/*Individual Color Addresses 
#define RedLowAddress 0x16
#define RedHighddress 0x17

#define BlueLowAddress 0x1A
#define BlueHighAddress 0x1B

#define GreenLowAddress 0x18
#define GreenHighAddress 0x19
*/


int running = 1;
#define MS 1000


//Initialize Registers 
uint8_t timee[2]; 
uint8_t gain[2];
uint8_t enable[2];
uint8_t colors[8];

/*
uint8_t red_low[2];
uint8_t red_high[2];

uint8_t blue_low[2];
uint8_t blue_high[2];

uint8_t green_low[2];
uint8_t green_high[2];
*/

void sig_handler(int signo) {
    if (signo == SIGINT) {
        printf("closing spi nicely\n");
        running = 0;
    }
}

void init_TCS34725(mraa::I2c *i2c) { 
	
	//Pass in Addresses and Data to Registers 
	
	timee[0] = ATimeAddress;
	timee[1] = 0xf6;

	gain[0] = ControlAddress;
	gain[1] = 0x00;

	enable[0] = EnableAddress;
	enable[1] = 0x03;

	assert(0 == i2c->address(SensorAddress)); 
	i2c->write(timee, 2); 
	i2c->write(gain, 2); 
	i2c->write(enable, 2); 
	/*mraa::printError(i2c->write(timee, 2)); 
	mraa::printError(i2c->write(gain, 2)); 
	mraa::printError(i2c->write(enable, 2));
	*/
}

void get_Colors(mraa::I2c *i2c){
	unsigned int clear_color = 0;
 	unsigned int red_color = 0;
 	unsigned int green_color = 0;
 	unsigned int blue_color = 0;

	i2c->address(SensorAddress); 
 	uint8_t color_values[8];
	for (i =0; i<8; i++){
		color_values[i] = i2c->read(colors, 1);
	}
 	printf("color values%d\n", color_values);

 	clear_color = (unsigned int)(color_values[1]<<8) + (unsigned int)color_values[0];
  	red_color = (unsigned int)(color_values[3]<<8) + (unsigned int)color_values[2];
  	green_color = (unsigned int)(color_values[5]<<8) + (unsigned int)color_values[4];
  	blue_color = (unsigned int)(color_values[7]<<8) + (unsigned int)color_values[6];


  	printf("clear_color:%d\n", (int)clear_color);
  	printf("red_color:%d\n", (int)red_color);
  	printf("blue_color%d\n", (int)blue_color);
  	printf("green_color%d\n", (int)green_color);
	/*
	i2c->address(SensorAddress); 
 	uint16_t clear_value = i2c->readWordReg(ColorAddress);
 	//printf("uint%d", color_value);
 	

 	i2c->address(SensorAddress); 
 	uint16_t red_value = i2c->readWordReg(ColorAddress + 2);
 	//printf("uint%d", color_value);
 	

	i2c->address(SensorAddress); 
 	uint16_t blue_value = i2c->readWordReg(ColorAddress + 4);
 	//printf("uint%d", color_value);
 	

 	i2c->address(SensorAddress); 
 	uint16_t green_value = i2c->readWordReg(ColorAddress + 6);
 	//printf("uint%d", color_value);
 	
 	*/
 }

int main(){
	    // Handle Ctrl-C quit
    signal(SIGINT, sig_handler);

    // Edison i2c bus is 6
    mraa::I2c *i2c = new mraa::I2c(6);
    assert(i2c != NULL);
    init_TCS34725(i2c);

    while (running){
    	get_Colors(i2c);
    }
}