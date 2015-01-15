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

uint8_t registers[2]; 

void sig_handler(int signo) {
    if (signo == SIGINT) {
        printf("closing spi nicely\n");
        running = 0;
    }
}

void init_TCS34725(mraa::I2c *i2c) { 
	
	i2c->address(SensorAddress); //Sensor Address

	assert(0 == i2c->writeReg(ATimeAddress, 0xf6));  //Integration Time
	assert(0 == i2c->writeReg(ControlAddress, 0x00)); //Gain
	assert(0 == i2c->writeReg(EnableAddress, 0x03)); //Colors?
}

void get_Colors(mraa::I2c *i2c){
	unsigned int clear_color = 0;
 	unsigned int red_color = 0;
 	unsigned int green_color = 0;
 	unsigned int blue_color = 0;

 	i2c->address(SensorAddress);

 	int color_value = int(i2c->readWordReg(ColorAddress));
 	printf("color_value\n");
 }

int main(){
	    // Handle Ctrl-C quit
    signal(SIGINT, sig_handler);

    // Edison i2c bus is 6
    mraa::I2c *i2c = new mraa::I2c(6);
    assert(i2c != NULL);
    init_TCS34725(i2c);

    while (running){
    	get_Colors();
    }
}