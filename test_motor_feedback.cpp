#include <cassert>
#include <cmath>
#include <csignal>
#include <iostream>
#include <sys/time.h>

#include "mraa.hpp"
//compile with: 
//g++ test_motor_feedback.cpp -o test_motor_feedback -lmraa

int running = 1;

void sig_handler(int signo)
{
  if (signo == SIGINT) {
    printf("closing spi nicely\n");
    running = 0;
  }
}

void setMotorSpeed(mraa::Pwm& pwm, mraa::Gpio& dir, double speed) {
  if(-1.0 >= speed){
    speed = -1;
  }
  if(speed >= 1.0){
    speed = 1;
  }
  assert(-1.0 <= speed && speed <= 1.0);
  if (speed < 0) {
    dir.write(1);
  }
  else {
    dir.write(0);
  }
  pwm.write(fabs(speed));
}

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
    distance = (distance*alpha) + (diffTime * 170.0 * (1.0 - alpha));
    std::cout << "Distance: " <<  distance << "m" << std::endl;
  }
}

int main() {
  // Handle Ctrl-C quit
  signal(SIGINT, sig_handler);

  // Ultrasound pins
  mraa::Gpio trig = mraa::Gpio(2);
  trig.dir(mraa::DIR_OUT);
  mraa::Gpio echo = mraa::Gpio(4);
  echo.dir(mraa::DIR_IN);
  // Set the echo handlers to receive rising or falling edges of the
  // echo pulse
  echo.isr(mraa::EDGE_BOTH, echo_handler, &echo);

  // Motor pins
  mraa::Pwm motPwm = mraa::Pwm(9);
  motPwm.write(0.0);
  motPwm.enable(true);
  mraa::Gpio motDir = mraa::Gpio(8);
  motDir.dir(mraa::DIR_OUT);
  motDir.write(0);

  //motor #2
  mraa::Pwm motPwm2 = mraa::Pwm(6);
  motPwm2.write(0.0);
  motPwm2.enable(true);
  mraa::Gpio motDir2 = mraa::Gpio(5);
  motDir2.dir(mraa::DIR_OUT);
  motDir2.write(0);

  while (running) {
    // 20us trigger pulse (must be at least 10us)
    trig.write(1);
    usleep(20);
    trig.write(0);

    if (distance > 0.2) {
      std::cout << "Setting motor speed: " << distance << std::endl;
      setMotorSpeed(motPwm, motDir, -1*(distance));
      setMotorSpeed(motPwm2, motDir2, distance);
    }
    else {
      setMotorSpeed(motPwm, motDir, distance);
      setMotorSpeed(motPwm2, motDir2, distance);
    }

    // Must pause at least 60ms between measurements
    usleep(200000.0);
  }
}

