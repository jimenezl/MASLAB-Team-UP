// Compile with:
// g++ test_color_sensor_with_arm.cpp -o test_color_sensor_with_arm -lmraa
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

#define SHIELD_I2C_ADDR 0x40
#define MS 1000

// Global Variables
int running = 1;

float cvalOne = 0;
float colorVal = 0;

float greenSwitch = 0;
float redSwitch = 0; 
bool servoRun = true;
bool notSorting = true; 


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

// Forward declarations of checkColors limitSwitches
void checkColors(float colorVal);
void limitSwitches(float switch1, float switch2, bool servoRun);

// Limit Switches
void limitSwitches(float switch1, float switch2, bool servoRun){

  if (switch1 < 1) {
    printf("Turning off motor\n");
    setMotorPosition(8, 0.0);

    if (servoRun){
      printf("Pushing red block\n");
      setServoPosition(15, 1.0); 
      printf("block pushed\n");
      sleep(1.0); //must be integer
      setServoPosition(15, -0.2); 
      printf("returning home\n"); 
      sleep(1.0); // return to home position
    }

  }
  else if (switch2 < 1){
    printf("Turning off motor\n");
    setMotorPosition(8, 0.0);
    
    if (servoRun){
      printf("Pushing green block\n");
      setServoPosition(15, 1.0);
      printf("block pushed\n");
      sleep(1.0);
      setServoPosition(15, -0.20); 
      printf("returning home\n");
      sleep(1.0); //return to home position
    }
  }
}

// Check color sensors and move to hopper
// green 490 < 550
// 370 < no block > 440
// 650 < red
void checkColors(float colorVal){
  if (colorVal > 650){ 
      printf("Red Block Found\n");
      servoRun = true;
      dirTurn.write(1);

      // adding in check for already being at red station
      if (redSwitch < 1){
        limitSwitches(redSwitch, greenSwitch, servoRun);
      }
      else {
        printf("Turntable moving\n");
        setMotorPosition(8, 0.12);
        sleep(2.0); // Give time for turntable to get to new pos.
        limitSwitches(greenSwitch, redSwitch, servoRun);
      }
    }
  else if (490 < colorVal && colorVal <= 550){ //prev. val<900 
      printf("Green Block Found\n");
      servoRun = true;
      dirTurn.write(0);
       // adding in check for already being at green station
      if (greenSwitch < 1){
        limitSwitches(greenSwitch, redSwitch, servoRun);
      }
      else { 
        printf("Turntable moving\n");
        setMotorPosition(8, 0.12);
        sleep(2.0); // Give time for turntable to get to new pos.
        limitSwitches(greenSwitch, redSwitch, servoRun);
      }
    }
  else if (colorVal < 440){
      printf("No Block Found\n"); //prev > 1000
      dirTurn.write(0);
      // adding in check for already being at green station
      if (greenSwitch < 1){
        limitSwitches(greenSwitch, redSwitch, servoRun);
      }
      else { 
        printf("Turntable moving\n");
        setMotorPosition(8, 0.12);
        sleep(2.0); // Give time for turntable to get to new pos.
        limitSwitches(greenSwitch, redSwitch, servoRun);
    }
  } 
}

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
    setServoPosition(0, 0.90);
    setMotorPosition(8, 0.0);
    printf("closing spi nicely\n");
    running = 0;
  }
}

int main() {
  // Handle Ctrl-C quit
  signal(SIGINT, sig_handler);

  // Color Sensor Readings to Pin 0
  // Limit Switch to Pin 1, Pin 2
  mraa::Aio colorSensor = mraa::Aio(0);
  mraa::Gpio limit1 = mraa::Gpio(1); //red side
  mraa::Gpio limit2 = mraa::Gpio(0); //green side

  mraa::Gpio armLimit = mraa::Gpio(2); // Arm Limit Switch
  bool armMoving = true;
  bool cubeFound = true;
  

  // Edison i2c bus is 6
  i2c = new mraa::I2c(6);
  assert(i2c != NULL);

  //Turntable motor
  dirTurn.dir(mraa::DIR_OUT);
  dirTurn.write(0);

  // Arm motor
  dirArm.dir(mraa::DIR_OUT);
  dirArm.write(0);
  usleep(1000*400);

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
  setMotorPosition(11, 0.2);
  usleep(1000*100);
  setMotorPosition(11, 0.0);
  printf("sleeping\n");
  sleep(10);
  setServoPosition(0, -0.2);
  sleep(2.0);

  // Move forwared 0.5 seconds
  speed = .2; 
  setMotorSpeed(pwm, dir, speed);
  setMotorSpeed(pwm2, dir2, -1*speed);
  usleep(500*1000);
  setMotorSpeed(pwm, dir, 0);
  setMotorSpeed(pwm2, dir2, 0);

  printf("starting\n");


  while (running) {
    int armVal = armLimit.read(); 
    colorVal = colorSensor.read();
    greenSwitch = limit1.read(); 
    redSwitch = limit2.read();

    if (cubeFound){ // Pick up Blocks
      dirArm.write(1);
      setServoPosition(0, 0.25); // close claw
      sleep(1.0);
      cubeFound = false;
    }

    if (armMoving){ // Arm moving up until switch hit
      setMotorPosition(11, 0.30);
    }

    if (armVal < 1){
      setMotorPosition(11, 0.0);
      setServoPosition(7,1.1);
      printf("Arm Limit: %d\n", armVal);
      armMoving = false;
      dirArm.write(0); 
      usleep(1000*300);
    }
    if (armMoving == false){

      // Hold arm up
      printf("Arm being held up\n");
      setServoPosition(7, 1.1);
      usleep(1000*300);
      setServoPosition(0, 0.7); //open up partially 
      sleep(2.0);

      // Sort blocks by color
      std::cout << "Colors: " << colorVal << std::endl;
      std::cout << "Red Switch: " << greenSwitch << std::endl;
      std::cout << "Green Switch: " << redSwitch << std::endl;  
      checkColors(colorVal); //checking color sensor
      sleep(3.0);
      
    }

  } 
    setMotorPosition(11, 0.0);
    setServoPosition(0, 0.90);
    setMotorPosition(8, 0.0);
}

