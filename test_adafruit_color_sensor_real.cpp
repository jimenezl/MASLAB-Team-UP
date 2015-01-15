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

int running = 1;
#define MS 1000

uint8_t timee[2]; 
uint8_t gain[2];
uint8_t enable[2];


void sig_handler(int signo) {
    if (signo == SIGINT) {
        printf("closing spi nicely\n");
        running = 0;
    }
}

void init_TCS34725(mraa::I2c *i2c) { 
	
	//Sensor Address
	
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
 	uint16_t clear_value = i2c->readWordReg(ColorAddress);
 	//printf("uint%d", color_value);
 	//printf("clear_value%d\n", (int)clear_value);

 	i2c->address(SensorAddress); 
 	uint16_t red_value = i2c->readWordReg(ColorAddress + 2);
 	//printf("uint%d", color_value);
 	//printf("red_value%d\n", (int)red_value);

	i2c->address(SensorAddress); 
 	uint16_t blue_value = i2c->readWordReg(ColorAddress + 4);
 	//printf("uint%d", color_value);
 	//printf("blue_value%d\n", (int)blue_value);

 	i2c->address(SensorAddress); 
 	uint16_t green_value = i2c->readWordReg(ColorAddress + 6);
 	//printf("uint%d", color_value);
 	//printf("green_value%d\n", (int)green_value);
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