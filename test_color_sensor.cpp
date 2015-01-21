// Compile with:
// g++ test_color_sensor.cpp -o test_color_sensor -lmraa
// Repeatedly reads pin A0 and prints the result.

#include "mraa.hpp"
#include <cassert>
#include <csignal>
#include <iostream>


// Global Variables
int running = 1;

int alpha = 0;
int cvalOne = 0;
int colorVal = 0;

int limitSwitch1 = 0;
int limitSwitch2 = 0; 
bool servoRun = true;

mraa::I2c *i2c;

mraa::Gpio dir = mraa::Gpio(3);

#define SHIELD_I2C_ADDR 0x40
#define MS 1000

// Motor Setup
uint8_t registers[] = {
    6,   // output 0
    10,  // output 1
    14,  // output 2
    18,  // output 3
    22,  // output 4
    26,  // output 5
    30,  // output 6
    34,  // output 7
    38,  // output 8
    42,  // output 9
    46,  // output 10
    50,  // output 11
    54,  // output 12
    58,  // output 13
    62,  // output 14
    66   // output 15
};

void initPWM() {
    uint8_t writeBuf[2] = {0};
    writeBuf[0] = 0x00; // Write to MODE 1 Register;
    writeBuf[1] = 1 << 4; // Enable Sleep Mode

    i2c->address(SHIELD_I2C_ADDR);
    i2c->write(writeBuf, 2);

    usleep(10 * MS);

    writeBuf[0] = 0xFE; // Write Prescaler Register
    writeBuf[1] = 0xA3; // Set pwm frequency to ~40 Hz

    i2c->address(SHIELD_I2C_ADDR);
    i2c->write(writeBuf, 2);

    writeBuf[0] = 0; // Write to the MODE 1 register
    writeBuf[1] = 1 << 5 // Enable auto increment mode
                  | 0 << 4; // Enable the oscillator

    i2c->address(SHIELD_I2C_ADDR);
    i2c->write(writeBuf, 2);
}


void writePWM(int index, double duty) {
    assert(-1.5 <= duty && duty <= 1.5);
    assert(0 <= index && index < 16);
    double on = 4095.0 * duty;
    uint16_t onRounded = (uint16_t) on;

    uint8_t writeBuf[5];

    // ON_L
    writeBuf[0] = registers[index];
    writeBuf[1] = 0x00; // ON LSB
    writeBuf[2] = 0x00; // ON MSB
    writeBuf[3] = onRounded & 0xFF; // OFF LSB
    writeBuf[4] = (onRounded >> 8) & 0xFF; // OFF MSB
    i2c->address(SHIELD_I2C_ADDR);
    i2c->write(writeBuf, 5);
}


void setServoPosition(int index, double duty) {
    writePWM(index, .04 * duty + .04);
}
void setMotorPosition(int index, double duty) {
    writePWM(index, duty);
}
// End Motor Setup

// Forward declarations of checkColors limitSwitches
void checkColors(int colorVal);
void limitSwitches(int switch1, int switch2, bool servoRun);

// Limit Switches
void limitSwitches(int switch1, int switch2, bool servoRun){

  if (switch1 > 100) {
    printf("Turning off motor\n");
    setMotorPosition(15, 0.01);

    if (servoRun){
      setServoPosition(1, 0.4);
      sleep(0.5);
      setServoPosition(1, -1.2); 
    }
    checkColors(colorVal);
  }
  else if (switch2 > 100){
    printf("Turning off motor\n");
    setMotorPosition(15, 0.01);
    
    if (servoRun){
      setServoPosition(1, 0.4);
      sleep(0.5);
      setServoPosition(1, -1.2);
    }
    checkColors(colorVal);
  }
}

// Check color sensors and move to hopper
void checkColors(int colorVal){
  if (colorVal > 750 && colorVal < 840){ //prev 900 to 1000
      printf("Red Block Found\n");
      dir.write(0);
      setMotorPosition(15, 0.15);
      limitSwitches(limitSwitch1, limitSwitch2, servoRun);
    }
  else if (colorVal <= 750){ //prev. val<900 
      printf("Green Block Found\n");
      dir.write(1);
      setMotorPosition(15, 0.15);
      limitSwitches(limitSwitch1, limitSwitch2, servoRun);
    }
  else {
      printf("No Block Found\n"); //prev > 1000
      dir.write(1);
      setMotorPosition(15, 0.15);
      servoRun = false; 
    }
}

void sig_handler(int signo)
{
  if (signo == SIGINT) {
    setMotorPosition(15, 0.0);
    printf("closing spi nicely\n");
    running = 0;
  }
}

int main() {
  // Handle Ctrl-C quit
  signal(SIGINT, sig_handler);

  //alpha for low pass filter
  alpha = 0.3;

  // Color Sensor Readings to Pin 0
  // Limit Switch to Pin 2, Pin 3
  mraa::Aio aio = mraa::Aio(0);
  mraa::Aio aio2 = mraa::Aio(2);
  mraa::Aio aio3 = mraa::Aio(3);

  // Edison i2c bus is 6
  i2c = new mraa::I2c(6);
  assert(i2c != NULL);

  //Turntable motor
  dir.dir(mraa::DIR_OUT);
  dir.write(0);

  initPWM();

  while (running) {
    cvalOne = aio.read();
    int cvalTwo = cvalOne; 
    colorVal = cvalTwo*alpha + cvalOne*(1.0 - alpha);
    limitSwitch1 = aio2.read();
    limitSwitch2 = aio3.read();

    std::cout << "Colors: " << colorVal << std::endl;
    std::cout << "Switch 1: " << limitSwitch1 << std::endl;
    std::cout << "Switch 2: " << limitSwitch2 << std::endl;

    checkColors(colorVal); //checking color sensor
    sleep(1.0);
  } 
}
