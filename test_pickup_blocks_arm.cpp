// Compile with:
// g++ test_pickup_blocks_arm.cpp -o test_pickup_blocks_arm -lmraa
// Picks up blocks and sorts them in sets of three
// brown, black, orange resistor for photoresistor
// red, red, brown resistor for led

// No Block: < 445
// Red: 450 < Red < 520
// Green: < 650 less than

#include "mraa.hpp"
#include <cassert>
#include <csignal>
#include <iostream>
#include <math.h>

#define SHIELD_I2C_ADDR 0x40
#define MS 1000

// Global Variables
int running = 1;

mraa::I2c *i2c;

mraa::Gpio dirTurn = mraa::Gpio(3); //Direction of Turntable
mraa::Gpio dirArm = mraa::Gpio(4); //Direction of Arm

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
    assert(0 <= duty && duty <= 1.0);
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
    //printf("Duty:\n", duty);
    writePWM(index, .04 * duty + .04);
}
void setMotorPosition(int index, double duty) {
    writePWM(index, duty);
}
// End Motor Setup

void setMotorSpeed(mraa::Pwm &pwm, mraa::Gpio &dir, double speed) {
    assert(-1.0 <= speed && speed <= 1.0);
    if (speed < 0) {
        dir.write(0);
    } else {
        dir.write(1);
    }
    pwm.write(fabs(speed));
}

void sig_handler(int signo)
{
  if (signo == SIGINT) {
    //turn Motors off or open them
    setMotorPosition(11, 0.0);
    setServoPosition(7, 1.6);
    setServoPosition(0, -0.20);

    printf("closing spi nicely\n");
    running = 0;
  }
}

int main() {
  // Handle Ctrl-C quit
  signal(SIGINT, sig_handler);

  mraa::Gpio armLimit = mraa::Gpio(2); // Arm Limit Switch

  // Edison i2c bus is 6
  i2c = new mraa::I2c(6);
  assert(i2c != NULL);

  // Arm motor
  dirArm.dir(mraa::DIR_OUT);
  dirArm.write(0); // Arm going down

  initPWM();

  mraa::Pwm pwm = mraa::Pwm(9);
  pwm.write(0.0);
  pwm.enable(true);
  //assert(pwm != NULL);
  mraa::Gpio dir = mraa::Gpio(8);
  //assert(dir != NULL);
  dir.dir(mraa::DIR_OUT);
  dir.write(1);
  
  mraa::Pwm pwm2 = mraa::Pwm(6);
  pwm2.write(0.0);
  pwm2.enable(true);
  //assert(pwm2 != NULL);
  mraa::Gpio dir2 = mraa::Gpio(5);
  //assert(dir != NULL);
  dir2.dir(mraa::DIR_OUT);
  dir2.write(1);

  // Arm Dropping and Claw Opening
  setServoPosition(7, 1.6);
  usleep(1000*400);
  printf("Dropping arm\n");
  setMotorPosition(11, 0.3);
  usleep(1000*1400);
  setMotorPosition(11, 0.0);
  printf("sleeping\n");
  sleep(2.0);
  setServoPosition(0, -0.2);
  sleep(2.0);

  // Move forwared 0.5 seconds
  float speed = .17; 
  printf("Moving Forward\n");
  setMotorSpeed(pwm, dir, -1*speed);
  setMotorSpeed(pwm2, dir2, speed);
  sleep(5.0);
  setMotorSpeed(pwm, dir, 0);
  setMotorSpeed(pwm2, dir2, 0);
  sleep(2.0);

  dirArm.write(1); // Arm going up
  printf("Claw closing\n");
  setServoPosition(0, 0.3); //Claw Closing
  sleep(1.0);

  while (running) {
    int armVal = 1;

    while(armVal > 0){ 
      armVal = armLimit.read();// Arm moving up until switch hit
      setMotorPosition(11, 0.25);
      printf("Arm Going up\n");
    }

    printf("Switch Hit!!\n");
    setMotorPosition(11, 0.0); ///turn off motor
    sleep(1.0);
    setServoPosition(7,1.1); // close gate
    printf("Arm Limit: %d\n", armVal);
    dirArm.write(0); 
    usleep(1000*300);
    
    printf("Dropping blocks\n");
    setServoPosition(0, 0.0); //open claw up partially 
    sleep(2.0);

    running = 0;  

  } 
    setMotorPosition(11, 0.0);
    setServoPosition(0, -0.20);
    setServoPosition(7, 1.1);
}

