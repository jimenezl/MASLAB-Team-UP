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
double currentDistanceEstimate = 10; //set to arbitrary distance
double distanceAlpha = .3;

mraa::Pwm pwmMotor1;
mraa::Gpio dir1;
double speed1;

mraa::Pwm pwmMotor2;
mraa::Gpio dir2;
double speed2;

void setMotorSpeed(mraa::Pwm& pwm, mraa::Gpio& dir, double speed) {
  assert(-1.0 <= speed && speed <= 1.0);
  if (speed < 0) {
    dir.write(1);
  }
  else {
    dir.write(0);
  }
  pwm.write(fabs(speed));
  std::cout << "Speed1: " << speed1< std::endl;
  std::cout << "Speed2: " << speed2< std::endl;
}

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
    currentDistanceEstimate = currentDistanceEstimate*distanceAlpha + (diffTime * 170.0 * (1.0 - distanceAlpha))
    // std::cout << "Distance: " <<  diffTime * 170.0 << "m" << std::endl;
    if (currentDistanceEstimate < .5){
      turnAround();
    }


  }
}

void turnAround(){
  setMotorSpeed(pwmMotor1, dir1, 0);
  setMotorSpeed(pwmMotor2, dir2, 0);

  setMotorSpeed(pwmMotor1, dir1, .5);
  setMotorSpeed(pwmMotor2, dir2, -.5);

  sleep(2.0);

  setMotorSpeed(pwmMotor1, dir1, .5);
  setMotorSpeed(pwmMotor2, dir2, .5);
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

  //Control Motors:
  pwmMotor1 = mraa::Pwm(9);
  pwmMotor1.write(0.0);
  pwmMotor1.enable(true);
  //assert(pwmMotor1 != NULL);
  dir1 = mraa::Gpio(8);
  //assert(dir1 != NULL);
  dir1.dir(mraa::DIR_OUT);
  dir1.write(0);
  
  double speed1 = -1.0;

  pwmMotor2 = mraa::Pwm(6);
  pwmMotor2.write(0.0);
  pwmMotor2.enable(true);
  //assert(pwmMotor2 != NULL);
  dir2 = mraa::Gpio(7);
  //assert(dir2 != NULL);
  dir2.dir(mraa::DIR_OUT);
  dir2.write(0);
  
  double speed1 = -1.0;

  //ultrasonic:
  mraa::Gpio* trig = new mraa::Gpio(2);
  trig->dir(mraa::DIR_OUT);
  mraa::Gpio* echo = new mraa::Gpio(4);
  echo->dir(mraa::DIR_IN);
  // Set the echo handlers to receive rising or falling edges of the
  // echo pulse
  echo->isr(mraa::EDGE_BOTH, echo_handler, echo);

  struct timespec lastUltrasonicMeasurementTime;
  clock_gettime( CLOCK_MONOTONIC, &lastUltrasonicMeasurementTime );
  struct timespec currentTime;
  clock_gettime(CLOCK_MONOTONIC, &currentTime );

  double ultrasonicDiffTime = double(currentTime.tv_sec - lastUltrasonicMeasurementTime.tv_sec) + (currentTime.tv_nsec - lastUltrasonicMeasurementTime.tv_nsec)/1000000000.0;

  while (running) {

    //ultrasonic measurements:
    clock_gettime(CLOCK_MONOTONIC, &currentTime);
    ultrasonicDiffTime = double(currentTime.tv_sec - lastUltrasonicMeasurementTime.tv_sec) + (currentTime.tv_nsec - lastUltrasonicMeasurementTime.tv_nsec)/1000000000.0;
    
    if (ultrasonicDiffTime > .06){
      // 20us trigger pulse (must be at least 10us)
      trig->write(1);
      usleep(20);
      trig->write(0);
      clock_gettime( CLOCK_MONOTONIC, &lastUltrasonicMeasurementTime );
  }

    
  }
}

