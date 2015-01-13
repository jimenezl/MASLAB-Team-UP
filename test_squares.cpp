// Build with:
// g++ test_squares.cpp -o test_squares -lmraa
// SPI pins are:
// - IO10: SS
// - IO11: MOSI
// - IO12: MISO
// - IO13: SCLK

#include <unistd.h>
#include <stdint.h>
#include <signal.h>
#include <sys/time.h>

#include <cassert>
#include <cmath>
#include <csignal>
#include <iostream>

#include "mraa.hpp"

#define MS 1000

#define GYRO_DATA_OKAY_MASK 0x0C000000
#define GYRO_DATA_OKAY 0x04000000

int running = 1;

void sig_handler(int signo)
{
  if (signo == SIGINT) {
    printf("closing spi nicely\n");
    running = 0;
  }
}

void setMotorSpeed(mraa::Pwm& pwm, mraa::Gpio& dir, double speed) {
  assert(-1.0 <= speed && speed <= 1.0);
  if (speed < 0) {
    dir.write(1);
  }
  else {
    dir.write(0);
  }
  pwm.write(fabs(speed));
}

int main()
{
  // Handle Ctrl-C quit
  signal(SIGINT, sig_handler);
  mraa::Gpio *chipSelect = new mraa::Gpio(10);
  chipSelect->dir(mraa::DIR_OUT);
  chipSelect->write(1);
  mraa::Spi* spi = new mraa::Spi(0);
  spi->bitPerWord(32);
  char rxBuf[2];
  char writeBuf[4];
  unsigned int sensorRead = 0x20000000;
  writeBuf[0] = sensorRead & 0xff;
  writeBuf[1] = (sensorRead >> 8) & 0xff;
  writeBuf[2] = (sensorRead >> 16) & 0xff;
  writeBuf[3] = (sensorRead >> 24) & 0xff;
  float total = 0;
  struct timeval tv;
  int init = 0;
  float rf;

  signal(SIGINT, sig_handler);

  //Motor Stuff
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

  struct timeval lastTurnTime;
  gettimeofday(&lastTurnTime, NULL);
  float startAngle = 0;
  bool isTurning = false;
  float speed = .5;
  float timeBetweenTurns = 3;
  setMotorSpeed(pwm, dir, -1*speed);
  setMotorSpeed(pwm2, dir2, speed);

  while (running) {
    chipSelect->write(0);
    char* recv = spi->write(writeBuf, 4);
    chipSelect->write(1);
//    printf("%x %x %x %x\r\n", recv[0], recv[1], recv[2], recv[3]);
    if (recv != NULL) {
      unsigned int recvVal = ((uint8_t) recv[3] & 0xFF);
      recvVal = (recvVal << 8) | ((uint8_t)recv[2] & 0xFF);
      recvVal = (recvVal << 8) | ((uint8_t)recv[1] & 0xFF);
      recvVal = (recvVal << 8) | ((uint8_t)recv[0] & 0xFF);
      printf("Received: 0x%.8x, ", recvVal);
      // Sensor reading
      short reading = (recvVal >> 10) & 0xffff;
      if (init) {
	unsigned long long ms = (unsigned long long)(tv.tv_sec)*1000 +
		(unsigned long long)(tv.tv_usec) / 1000;
	gettimeofday(&tv, NULL);
	ms -= (unsigned long long)(tv.tv_sec)*1000 +
		(unsigned long long)(tv.tv_usec) / 1000;
	int msi = (int)ms;
	float msf = (float)msi;
	rf = (float)reading;
        total += -0.001 * msf * (rf / 80.0);
        printf("Total: %f, Reading: %f, Time: %f\n", total, rf, -msf);
      }
      else {
	init = 1;
	gettimeofday(&tv, NULL);
      }
    }
    else {
      printf("No recv\n");
    }
    usleep(10 * MS);

    if (((tv.tv_sec - lastTurnTime.tv_sec)> timeBetweenTurns) && (!isTurning)){
      startAngle = rf;
      setMotorSpeed(pwm, dir, speed);
      setMotorSpeed(pwm2, dir2, speed);
      isTurning = true;
    }

    if (isTurning){
      if ((rf - startAngle) >= 90.0){
        isTurning = false;
        setMotorSpeed(pwm, dir, -1*speed);
        setMotorSpeed(pwm2, dir2, speed);
        gettimeofday(&lastTurnTime, NULL);
      }
      else {

      }
    }

  }

  delete spi;

  return 0;
}
