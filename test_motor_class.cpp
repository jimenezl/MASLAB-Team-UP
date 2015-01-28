// Compile with:
// g++ test_motor_class.cpp -o test_motor_class -lmraa
// Controls a motor through a range of speeds using the Cytron motor controller
// Pwm on pin 9, and dir on pin 8.

#include <cassert>
#include <cmath>
#include <csignal>
#include <iostream>

#include "mraa.hpp"

int running = 1;

void sig_handler(int signo) {
    if (signo == SIGINT) {
        printf("closing spi nicely\n");
        running = 0;
    }
}

class WheelController {
public:
    int PWM_ONE_PIN = 9;
    int DIR_ONE_PIN = 8;
    
    int PWM_TWO_PIN = 6;
    int DIR_TWO_PIN = 5;

    mraa::Pwm pwm;
    mraa::Gpio dir;

    mraa::Pwm pwm2;
    mraa::Gpio dir2;

    void setMotorSpeed(mraa::Pwm &pwm, mraa::Gpio &dir, double speed) {
        assert(-1.0 <= speed && speed <= 1.0);
        if (speed < 0) {
            dir.write(1);
        } else {
            dir.write(0);
        }
        pwm.write(fabs(speed));
    }

    void init() {
        pwm = mraa::Pwm(9);
        pwm.write(0.0);
        pwm.enable(true);
        //assert(pwm != NULL);
        dir = mraa::Gpio(8);
        //assert(dir != NULL);
        dir.dir(mraa::DIR_OUT);
        dir.write(0);

        pwm2 = mraa::Pwm(6);
        pwm2.write(0.0);
        pwm2.enable(true);
        //assert(pwm2 != NULL);
        dir2 = mraa::Gpio(5);
        //assert(dir != NULL);
        dir2.dir(mraa::DIR_OUT);
        dir2.write(0);

    }

    void setMotorOneSpeed(double speed){
        setMotorSpeed(pwm, dir, speed);
    }

    void setMotorTwoSpeed(double speed){
        setMotorSpeed(pwm2, dir2, speed);
    }

} ;

int main() {
    // Handle Ctrl-C quit
    signal(SIGINT, sig_handler);

    WheelController wheelController(void);
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

