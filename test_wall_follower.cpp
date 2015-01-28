// Build with:
// g++ test_wall_follower.cpp -o test_wall_follower -lmraa
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
#include <math.h>

#include "mraa.hpp"

#define MS 1000

#define GYRO_DATA_OKAY_MASK 0x0C000000
#define GYRO_DATA_OKAY 0x04000000

#define PI 3.14159265

float DISTANCE_FROM_IR_SENSORS = 4.6; //
// float QUAD_TERM = ;
// float LINEAR_TERM = ;
// float CONST_TERM = ;
int BACK_INFRARED_PIN = 3;
int FRONT_INFRARED_PIN = 2;

float alpha = .3;

int running = 1;

void sig_handler(int signo) {
    if (signo == SIGINT) {
        printf("closing spi nicely\n");
        mraa::Pwm pwm = mraa::Pwm(9);
        mraa::Pwm pwm2 = mraa::Pwm(6);
        pwm.write(0);
        pwm2.write(0);
        running = 0;
    }
}

void setMotorSpeed(mraa::Pwm &pwm, mraa::Gpio &dir, double speed) {
    assert(-1.0 <= speed && speed <= 1.0);
    if (speed < 0) {
        dir.write(1);
    } else {
        dir.write(0);
    }
    pwm.write(fabs(speed));
}

float angleFromWall(float backInfraDistance, float frontInfraDistance){
    bool inQuadrantOne = true;
    if (backInfraDistance > frontInfraDistance){
        inQuadrantOne = true;
    } else {
        inQuadrantOne = false;
    }

    float diffDistance = fabs(backInfraDistance - frontInfraDistance);

    if(inQuadrantOne){
        return asin(diffDistance/DISTANCE_FROM_IR_SENSORS);
    } else{
        return -1.0 * acos(diffDistance/DISTANCE_FROM_IR_SENSORS);
    }
}

float infraReadingToDistanceBack(float infraReading){
    // return (QUAD_TERM * infraReading * infraReading) + (LINEAR_TERM * infraReading) + CONST_TERM;
    if (infraReading!=0){
        return (970.0/infraReading) - .5; //y = 970/x fits our data 
    } else {
        return 10.0; //big number
    }
}

float infraReadingToDistanceFront(float infraReading){
    // return (QUAD_TERM * infraReading * infraReading) + (LINEAR_TERM * infraReading) + CONST_TERM;
    if (infraReading!=0){
        return (970.0/infraReading) - .5;; //y = 970/x  - .5
    } else {
        return 10.0; //big number
    }
}

int main() {
    // Handle Ctrl-C quit
    signal(SIGINT, sig_handler);
    // mraa::Gpio *chipSelect = new mraa::Gpio(10);
    // chipSelect->dir(mraa::DIR_OUT);
    // chipSelect->write(1);
    // mraa::Spi *spi = new mraa::Spi(0);
    // spi->bitPerWord(32);
    // char rxBuf[2];
    // char writeBuf[4];
    // unsigned int sensorRead = 0x20000000;
    // writeBuf[0] = sensorRead & 0xff;
    // writeBuf[1] = (sensorRead >> 8) & 0xff;
    // writeBuf[2] = (sensorRead >> 16) & 0xff;
    // writeBuf[3] = (sensorRead >> 24) & 0xff;
    // float total = 0;
    // struct timeval tv;
    // int init = 0;
    // float rf;

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
    mraa::Gpio dir2 = mraa::Gpio(5);
    //assert(dir != NULL);
    dir2.dir(mraa::DIR_OUT);
    dir2.write(0);

    mraa::Aio aioBackInfrared = mraa::Aio(BACK_INFRARED_PIN);
    mraa::Aio aioFrontInfrared = mraa::Aio(FRONT_INFRARED_PIN);

    float speed = .1;
    float desiredAngle = 0.0;
    float diffAngle = 0.0;
    float integral = 0;
    float power = 0;
    float derivative = 0;
    float timeBetweenReadings = 0;
    float gyroBias = 1.0;
    float forwardBias = 0;
    float P_CONSTANT = 25;
    float I_CONSTANT = 0;
    float D_CONSTANT = -1;

    float backDistance = 3.0;
    float frontDistance = 3.0;

    float P_CONSTANT_WALL_FOLLOWER = .05 * 180.0 / PI;

    while (running) {

        float backInfraredReading = aioBackInfrared.read();
        float frontInfraredReading = aioFrontInfrared.read();
        // printf("Infra readings: back: %f, front: %f\n", backInfraredReading, frontInfraredReading);

        backDistance =  (backDistance * alpha) + (infraReadingToDistanceBack(backInfraredReading) * (1.0 - alpha));
        frontDistance = (frontDistance * alpha) + (infraReadingToDistanceFront(frontInfraredReading) * (1.0 - alpha) * .94);
        printf("Distances: Front: %f, Back: %f\n", backDistance, frontDistance);
        float infraAngle = angleFromWall(backDistance, frontDistance);
        printf("estimated angle: %f\n", infraAngle);
        // power = speed * ((P_CONSTANT * diffAngle / 360.0) + (I_CONSTANT * integral) + (D_CONSTANT * derivative / 180.0)); //make sure to convert angles > 360 to proper angles
        power = speed * (P_CONSTANT_WALL_FOLLOWER * infraAngle);

        if (fabs(infraAngle) < 5.0){
            power = 0;
        }

        if (power > .3) {
            power = .3;
        } else if (power < -.3) {
            power = -.3;
        }
        setMotorSpeed(pwm, dir, -1 * power + forwardBias);
        setMotorSpeed(pwm2, dir2, -1 * power - forwardBias);
        printf("Set power to: %f\n", power);

    }

    return 0;
}
