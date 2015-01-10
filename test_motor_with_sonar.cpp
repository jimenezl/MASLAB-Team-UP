// Compile with:
// g++ test_motor.cpp -o test_motor -lmraa
// Controls a motor through a range of speeds using the Cytron motor controller
// Pwm on pin 9, and dir on pin 8.

#include <cassert>
#include <cmath>
#include <csignal>
#include <iostream>
#include <sys/time.h>

#include "mraa.hpp"

int running = 1;

static double distance = -1.0;
static double alpha = .7;

void echo_handler(void* args) {
  // Grab end time first, for accuracy
  struct timeval end;
  gettimeofday(&end, NULL);

  mraa::Gpio* echo = (mraa::Gpio*)args;
  static struct timeval start;
  bool rising = echo->read() == 1;
  if (rising) {
    gettimeofday(&start, NULL);
  }
  else {
    int diffSec = end.tv_sec - start.tv_sec;
    std::cout << "Diff sec: " << diffSec << std::endl;
    int diffUSec = end.tv_usec - start.tv_usec;
    std::cout << "Diff usec: " << diffUSec << std::endl;
    double diffTime = (double)diffSec + 0.000001*diffUSec;
    std::cout << "Diff time: " << diffTime << std::endl;
    // Speed of sound conversion: 340m/s * 0.5 (round trip)
    if ((diffTime * 170.0 ) < 5){
      distance = (distance*alpha) + (diffTime * 170.0 * (1.0 - alpha));
  }
    std::cout << "Distance: " <<  distance << "m" << std::endl;
  }
}

void setMotorSpeed(mraa::Pwm& pwm, mraa::Gpio& dir, double speed) {
  if(-1.0 >= speed){
    speed = -1.0;
  }
  if(1.0 <= speed){
    speed = 1.0;
  }
  // assert(-1.0 <= speed && speed <= 1.0);
  if (speed < 0) {
    dir.write(1);
  }
  else {
    dir.write(0);
  }
  pwm.write(fabs(speed));
}

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

  mraa::Pwm pwm = mraa::Pwm(9);
  pwm.write(0.0);
  pwm.enable(true);
  //assert(pwm != NULL);
  mraa::Gpio dir = mraa::Gpio(8);
  //assert(dir != NULL);
  dir.dir(mraa::DIR_OUT);
  dir.write(0);

  mraa::Pwm pwm2 = mraa::Pwm(6);
  pwm2.write(0.0);
  pwm2.enable(true);
  //assert(pwm2 != NULL);
  mraa::Gpio dir2 = mraa::Gpio(7);
  //assert(dir != NULL);
  dir2.dir(mraa::DIR_OUT);
  dir2.write(0);

  // Ultrasound pins
  mraa::Gpio trig = mraa::Gpio(2);
  trig.dir(mraa::DIR_OUT);
  mraa::Gpio echo = mraa::Gpio(4);
  echo.dir(mraa::DIR_IN);
  // Set the echo handlers to receive rising or falling edges of the
  // echo pulse
  echo.isr(mraa::EDGE_BOTH, echo_handler, &echo);
  
  double speed = 1.0;
  while (running) {

    // 20us trigger pulse (must be at least 10us)
    trig.write(1);
    usleep(20);
    trig.write(0);

    speed = distance;

    if (distance < .2) {
      speed = -.5;
    }
    
    std::cout << "Speed: " << speed << std::endl;
    setMotorSpeed(pwm, dir, speed);
    setMotorSpeed(pwm2, dir2, -1*speed);

    
    usleep(100000);

    // Must pause at least 60ms between measurements
    usleep(200000.0);
    
  }
}

