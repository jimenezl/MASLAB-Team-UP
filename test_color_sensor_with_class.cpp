// Compile with:
// g++ test_color_sensor_with_class.cpp -o test_color_sensor_with_class -lmraa
// Picks up blocks and sorts them in sets of three
// brown, black, orange resistor for photoresistor
// red, red, brown resistor for led



#include "mraa.hpp"
#include <cassert>
#include <csignal>
#include <iostream>

#include "ColorSensor.cpp"

// Global Variables
int running = 1;

void sig_handler(int signo)
{
  if (signo == SIGINT) {
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
    lilMama.readValues();

    if (lilMama.cubeFound){ // Pick up Blocks
      lilMama.dirArm.write(0);
      lilMama.setServoPosition(0, 0.40);
      printf("close gripper\n");
      sleep(1.0);
      lilMama.cubeFound = false;
    }
    if (lilMama.armMoving){  // Arm moving up until switch hit
      lilMama.setMotorPosition(11, 0.30);
      printf("Arm Moving Up\n");
      printf("Arm Limit: %d\n", lilMama.armVal);
      }

    if (lilMama.armVal < 1){
      if (lilMama.notSorting){
      lilMama.armMoving = false;

      // Turn motor off
      printf("Arm Limit: %d\n", lilMama.armVal);
      lilMama.setMotorPosition(11, 0.0);
      lilMama.dirArm.write(1);
      usleep(1000*500);

      // Hold arm up
      printf("Arm being held up\n");
      lilMama.setServoPosition(4, 1.1);
      sleep(2.0);
      lilMama.setServoPosition(0, 0.70);
      sleep(2.0);
      }
      // Sort blocks by color
      lilMama.notSorting = false;
      std::cout << "Colors: " << lilMama.colorVal << std::endl;
      std::cout << "Switch 1: " << lilMama.greenSwitch << std::endl;
      std::cout << "Switch 2: " << lilMama.redSwitch << std::endl;
      lilMama.checkColors(lilMama.colorVal); //checking color sensor
      sleep(3.0);
      }
    }

  } 

