class WheelController {
public:
    int PWM_ONE_PIN = 9;
    int DIR_ONE_PIN = 8;
    
    int PWM_TWO_PIN = 6;
    int DIR_TWO_PIN = 5;

    mraa::Pwm pwm = mraa::Pwm(PWM_ONE_PIN);
    mraa::Gpio dir = mraa::Gpio(DIR_ONE_PIN);

    mraa::Pwm pwm2 = mraa::Pwm(PWM_TWO_PIN);
    mraa::Gpio dir2 = mraa::Gpio(DIR_TWO_PIN);

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
        // pwm = mraa::Pwm(PWM_ONE_PIN);
        pwm.write(0.0);
        pwm.enable(true);
        //assert(pwm != NULL);
        // dir = mraa::Gpio(DIR_ONE_PIN);
        //assert(dir != NULL);
        dir.dir(mraa::DIR_OUT);
        dir.write(0);

        // pwm2 = mraa::Pwm(PWM_TWO_PIN);
        pwm2.write(0.0);
        pwm2.enable(true);
        //assert(pwm2 != NULL);
        // dir2 = mraa::Gpio(DIR_TWO_PIN);
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