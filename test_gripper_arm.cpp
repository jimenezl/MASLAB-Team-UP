// Compile with:
// g++ test_gripper_arm.cpp -o test_gripper_arm -lmraa
// Repeatedly reads pin A0 and prints the result.
// brown, black, orange resistor for photoresistor
// red, red, brown resistor for led

#include "mraa.hpp"
#include <cassert>
#include <csignal>
#include <iostream>

/* 
Servos 
 15 pusher
 7 door
 0 gripper

Drive motors
 11 motor arm
 8 motor turntable 

Analogs 
 1 red limit switch 1
 2 green limit switch 2
 0 Photoresistor
 3 right rear IR sensor 
 4 front right IR sensor 
 5 front IR sensor 
*/
    
// Global Variables
int running = 1;

mraa::I2c *i2c;
mraa::Gpio dir = mraa::Gpio(4);



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


void sig_handler(int signo)
{
  if (signo == SIGINT) {
    setMotorPosition(11, 0.0);
    printf("closing spi nicely\n");
    running = 0;
  }
}

int main() {
  // Handle Ctrl-C quit
  signal(SIGINT, sig_handler);

  mraa::Gpio armLimit = mraa::Gpio(2);
  bool armMoving = true;
    // Edison i2c bus is 6
  i2c = new mraa::I2c(6);
  assert(i2c != NULL);

  //Turntable motor
  dir.dir(mraa::DIR_OUT);
  dir.write(1);

  initPWM();

  while (running) {
    int armVal = armLimit.read();
    if (armMoving){
    printf("Arm Limit: %d\n", armVal);
    dir.write(1);
    setServoPosition(0, -0.10);
    printf("close gripper\n");
    sleep(1.0);
}
    setMotorPosition(11, 0.30);
    printf("Arm Moving Up\n");
    armMoving = false; 
    
    printf("arm going up\n");
    if (armVal < 1){
        printf("Arm Limit: %d\n", armVal);
        setMotorPosition(11, 0.0);
        sleep(1.0);
        setServoPosition(0, 0.20);
        sleep(2.0);
        
      // arm going down
        dir.write(0);
        setMotorPosition(11, 0.2);
        sleep(4.0);
        setServoPosition(0, 1.0);
        sleep(2.0);
        armMoving = true;
    }
  }

}