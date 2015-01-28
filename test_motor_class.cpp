// Compile with:
// g++ test_motor_class.cpp -o test_motor_class -lmraa -std=c++11
// Controls a motor through a range of speeds using the Cytron motor controller
// Pwm on pin 9, and dir on pin 8.

#include <cassert>
#include <cmath>
#include <csignal>
#include <iostream>

#include "mraa.hpp"
#include "WheelController.cpp"

int running = 1;

void sig_handler(int signo) {
    if (signo == SIGINT) {
        printf("closing spi nicely\n");
        running = 0;
    }
}



int main() {
    // Handle Ctrl-C quit
    signal(SIGINT, sig_handler);

    WheelController wheelController;
    wheelController.init();

    double speed = -1.0;
    while (running) {
        double speed = 0.3;
        std::cout << "Speed: " << speed << std::endl;
        wheelController.setMotorOneSpeed(speed);
        wheelController.setMotorTwoSpeed(speed);
        // setMotorSpeed(pwm, dir, speed);
        //setMotorSpeed(pwm2, dir2, -1 * speed);

        /*speed += 0.1;
        if (speed > 1.0) {
            speed = -1.0;
            // Let the motor spin down
            setMotorSpeed(pwm, dir, 0.0);
            setMotorSpeed(pwm2, dir2, 0.0);
            sleep(2.0);
        
        }
        */
        usleep(100000);

    }
}

