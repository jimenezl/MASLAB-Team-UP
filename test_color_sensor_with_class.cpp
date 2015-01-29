// Compile with:
// g++ test_color_sensor.cpp -o test_color_sensor -lmraa
// Repeatedly reads pin A0 and prints the result.
// brown, black, orange resistor for photoresistor
// red, red, brown resistor for led



#include "mraa.hpp"
#include <cassert>
#include <csignal>
#include <iostream>

#include "ColorSensor.cpp"

// Global Variables
int running = 1;


#define SHIELD_I2C_ADDR 0x40
#define MS 1000

void sig_handler(int signo)
{
  if (signo == SIGINT) {
    setMotorPosition(8, 0.0);
    printf("closing spi nicely\n");
    running = 0;
  }
}

int main() {
  // Handle Ctrl-C quit
  signal(SIGINT, sig_handler);

  ColorSensor lilMama;
  lilMama.init();
  lilMama.initPWM();

  while (running) {
    int armVal = armLimit.read(); 
    colorVal = colorSensor.read();
    greenSwitch = limit1.read(); //Green block canister
    redSwitch = limit2.read();

    if (cubeFound){ // Arm moving up until switch hit
      printf("Arm Limit: %d\n", armVal);
      dirArm.write(1);
      lilMama.setServoPosition(0, 0.40);
      printf("close gripper\n");
      sleep(1.0);
    }
    if (armMoving){
      lilMama.setMotorPosition(11, 0.30);
      printf("Arm Moving Up\n");
      }
      
    if (armVal < 1){
      armMoving = false;
      cubeFound = false;

      printf("Arm being held up\n");
      lilMama.setServoPosition(4, 1.1);

      std::cout << "Colors: " << colorVal << std::endl;
      std::cout << "Switch 1: " << greenSwitch << std::endl;
      std::cout << "Switch 2: " << redSwitch << std::endl;
      
      printf("Arm Limit: %d\n", armVal);
      lilMama.setMotorPosition(11, 0.0);
      dirArm.write(0);
      sleep(2.0);
      lilMama.setServoPosition(0, 0.70);
      sleep(2.0);
      servoRun = true;

      while(servoRun){
        lilMama.checkColors(colorVal); //checking color sensor
        sleep(3.0);
      }
    }

  } 
}
