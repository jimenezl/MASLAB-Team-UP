// Build with:
// g++ test_wall_follower_distance.cpp -o test_wall_follower_distance -lmraa
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

float DISTANCE_FROM_IR_SENSORS = 4.6;
int BACK_INFRARED_PIN = 3;
int FRONT_INFRARED_PIN = 2;
int HEAD_INFRARED_PIN = 1;

float alpha_infrareds = .3;

int running = 1;

void sig_handler(int signo) {
    if (signo == SIGINT) {
        printf("closing spi nicely\n");
        running = 0;
        mraa::Pwm pwm = mraa::Pwm(9);
        mraa::Pwm pwm2 = mraa::Pwm(6);
        pwm.write(0);
        pwm2.write(0);
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
        return (970.0/infraReading) - .5; //y = 970/x - .3 fits our data 
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

float infraReadingToDistanceHead(float infraReading){
    // return (QUAD_TERM * infraReading * infraReading) + (LINEAR_TERM * infraReading) + CONST_TERM;
    if (infraReading!=50){
        return (900.0/(infraReading - 50.0)); //y = 900/(x-50)
    } else {
        return 10.0; //big number
    }
}

int main() {
    // Handle Ctrl-C quit
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
    mraa::Aio aioHeadInfrared = mraa::Aio(HEAD_INFRARED_PIN);

    float speedWallFollower = .1;
    float powerWallFollower = 0;
    float forwardBiasWallFollower = .15;

    float backDistance = 0;
    float frontDistance = 0;
    float headDistance = 10.0;

    float desiredDistance = 5;

    float P_CONSTANT_WALL_FOLLOWER = .15;

    while (running) {

        float backInfraredReading = aioBackInfrared.read();
        float frontInfraredReading = aioFrontInfrared.read();
        float headInfraredReading = aioHeadInfrared.read();
        printf("Infra readings: back: %f, front: %f, head: %f\n", backInfraredReading, frontInfraredReading, headInfraredReading);

        backDistance =  (backDistance * alpha_infrareds) + (infraReadingToDistanceBack(backInfraredReading) * (1.0 - alpha_infrareds));
        frontDistance = (frontDistance * alpha_infrareds) + (infraReadingToDistanceFront(frontInfraredReading) * (1.0 - alpha_infrareds) * .94);
        headDistance = (headDistance * alpha_infrareds) + (infraReadingToDistanceHead(headInfraredReading) * (1.0 - alpha_infrareds));
        printf("Distances: Back: %f, Front: %f, Head: %f\n", backDistance, frontDistance, headDistance);
        float averageDistance = (backDistance + frontDistance) / 2.0;
        
        float diffDistance = desiredDistance - averageDistance;
        powerWallFollower = speedWallFollower * (P_CONSTANT_WALL_FOLLOWER * diffDistance);

        if (powerWallFollower > .3) {
            powerWallFollower = .3;
        } else if (powerWallFollower < -.3) {
            powerWallFollower = -.3;
        }
        if (headDistance < 6.0 && headDistance > 0){
            printf("PowerWallFollower reduced!\n"); //head hit a wall
            powerWallFollower = .2;
            setMotorSpeed(pwm, dir, powerWallFollower);
            setMotorSpeed(pwm2, dir2, powerWallFollower);
            usleep(1000 * 20); 
        } else if ((backDistance > 20) && (frontDistance < 20 && frontDistance > 0 )){
            powerWallFollower = .15; //only front distance gives good readings, turn left
            setMotorSpeed(pwm, dir, powerWallFollower + forwardBiasWallFollower);
            setMotorSpeed(pwm2, dir2, powerWallFollower - forwardBiasWallFollower);
        } else if ((frontDistance > 20) && (backDistance < 20 && backDistance > 0 )){
            setMotorSpeed(pwm, dir, forwardBiasWallFollower);
            setMotorSpeed(pwm2, dir2, -1.0 * forwardBiasWallFollower);
            usleep(300 * 1000);
            powerWallFollower = -.15; //only back distance gives good readings, turn right
            setMotorSpeed(pwm, dir, powerWallFollower + forwardBiasWallFollower);
            setMotorSpeed(pwm2, dir2, powerWallFollower - forwardBiasWallFollower);
            usleep(300 * 1000);
        } else if ((frontDistance > 20 || frontDistance < 0) && (backDistance > 20 || backDistance < 0)) {
            powerWallFollower = .15; //sensors read garbage
            setMotorSpeed(pwm, dir, powerWallFollower);
            setMotorSpeed(pwm2, dir2, -1 * powerWallFollower);
        }
        else if ((frontDistance < 20 && frontDistance > 0) && (backDistance < 20 && backDistance > 0)) {
            setMotorSpeed(pwm, dir, powerWallFollower + forwardBiasWallFollower); //normal behavior
            setMotorSpeed(pwm2, dir2, powerWallFollower - forwardBiasWallFollower);   
        } else {
            setMotorSpeed(pwm, dir, 0);
            setMotorSpeed(pwm2, dir2, 0);
            running = 0;
        }
        printf("Set power to: %f\n", powerWallFollower);

    }

    setMotorSpeed(pwm, dir, 0);
    setMotorSpeed(pwm2, dir2, 0);
    return 0;
}
