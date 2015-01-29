class ColorSensor {
public:

		float cvalOne = 0;
		float colorVal = 0;
		int armVal = 0;

		float greenSwitch = 0;
		float redSwitch = 0; 
		bool servoRun = true;

		mraa::Aio colorSensor = mraa::Aio(0);
		mraa::Gpio limit1 = mraa::Gpio(1); // Green Switch
		mraa::Gpio limit2 = mraa::Gpio(0); // Red Switch
		mraa::Gpio armLimit = mraa::Gpio(2); // Arm Limit Switch
		bool armMoving = true;
		bool cubeFound = true;

		// Edison i2c bus is 6
		mraa::I2c *i2c;
		

		
		mraa::Gpio dirTurn = mraa::Gpio(3); //Direction of Turntable
		mraa::Gpio dirArm = mraa::Gpio(4); //Direction of Arm

		#define SHIELD_I2C_ADDR 0x40
		#define MS 1000

		// Motor Setup
		uint8_t registers[16] = {
		    6,   // output 0
		    10,  // output 1
		    14,  // output 2
		    18,  // output 3
		    22,  // output 4
		    26,  // output 5
		    30,  // output 6
		    34,  // output 7
		    38,  // output 8
		    42,  // output 9
		    46,  // output 10
		    50,  // output 11
		    54,  // output 12
		    58,  // output 13
		    62,  // output 14
		    66   // output 15
		};

	void initPWM() {
	    uint8_t writeBuf[2] = {0};
	    writeBuf[0] = 0x00; // Write to MODE 1 Register;
	    writeBuf[1] = 1 << 4; // Enable Sleep Mode

	    i2c->address(SHIELD_I2C_ADDR);
	    i2c->write(writeBuf, 2);

	    usleep(10 * MS);

	    writeBuf[0] = 0xFE; // Write Prescaler Register
	    writeBuf[1] = 0xA3; // Set pwm frequency to ~40 Hz

	    i2c->address(SHIELD_I2C_ADDR);
	    i2c->write(writeBuf, 2);

	    writeBuf[0] = 0; // Write to the MODE 1 register
	    writeBuf[1] = 1 << 5 // Enable auto increment mode
	                  | 0 << 4; // Enable the oscillator

	    i2c->address(SHIELD_I2C_ADDR);
	    i2c->write(writeBuf, 2);
	}

	void writePWM(int index, double duty) {
	    assert(0 <= duty && duty <= 1.0);
	    assert(0 <= index && index < 16);
	    double on = 4095.0 * duty;
	    printf("duty:%f\n", duty);
	    
	    uint16_t onRounded = (uint16_t) on;

	    uint8_t writeBuf[5];

	    // ON_L
	    writeBuf[0] = registers[index];
	    writeBuf[1] = 0x00; // ON LSB
	    writeBuf[2] = 0x00; // ON MSB
	    writeBuf[3] = onRounded & 0xFF; // OFF LSB
	    writeBuf[4] = (onRounded >> 8) & 0xFF; // OFF MSB
	    i2c->address(SHIELD_I2C_ADDR);
	    i2c->write(writeBuf, 5);
	}

	void setServoPosition(int index, double duty) {
    //printf("Duty:\n", duty);
    writePWM(index, .04 * duty + .04);
	}
	void setMotorPosition(int index, double duty) {
    writePWM(index, duty);
	}

	/*// Forward declarations of checkColors limitSwitches
	void checkColors(float colorVal);
	void limitSwitches(float switch1, float switch2, bool servoRun);
*/
	// Limit Switches
	void limitSwitches(float switch1, float switch2, bool servoRun){

	  if (switch1 < 1000) {
	    printf("Turning off motor\n");
	    setMotorPosition(8, 0.0);

	    if (servoRun){
	      printf("Pushing green block\n");

	      setServoPosition(15, 1.0); 
	      printf("block pushed\n");
	      sleep(1.0); //must be integer
	      setServoPosition(15, -0.2); 
	      printf("returning home\n"); 
	      sleep(1.0); // return to home position
	    }

	  }
	  else if (switch2 < 1000){
	    printf("Turning off motor\n");
	    setMotorPosition(8, 0.0);
	    
	    if (servoRun){
	      printf("Pushing red block\n");

	      setServoPosition(15, 1.0);
	      printf("block pushed\n");
	      sleep(1.0);
	      setServoPosition(15, -0.20); 
	      printf("returning home\n");
	      sleep(1.0); //return to home position
	    }
	  }
	}

	// Check color sensors and move to hopper
	// green <340
	// no block > 760
	// 460 red
	void checkColors(float colorVal){
	  if (colorVal > 400 && colorVal < 700){ //prev 900 to 1000
	      printf("Red Block Found\n");
	      servoRun = true;
	      dirTurn.write(0);

	      // adding in check for already being at red station
	      if (redSwitch < 1000){
	        limitSwitches(greenSwitch, redSwitch, servoRun);
	      }
	      else {
	        setMotorPosition(8, 0.15);
	        sleep(2.0);
	        limitSwitches(greenSwitch, redSwitch, servoRun);
	      }
	    }
	  else if (colorVal <= 400){ //prev. val<900 
	      printf("Green Block Found\n");
	      servoRun = true;
	      dirTurn.write(1);
	       // adding in check for already being at green station
	      if (greenSwitch < 1000){
	        limitSwitches(greenSwitch, redSwitch, servoRun);
	      }
	      else { 
	      setMotorPosition(8, 0.15);
	      sleep(2.0);
	      limitSwitches(greenSwitch, redSwitch, servoRun);
	      }
	    }
	  else if (colorVal >= 700){
	      printf("No Block Found\n"); //prev > 1000
	      dirTurn.write(1);
	      // adding in check for already being at green station
	      if (greenSwitch < 1000){
	      setMotorPosition(8, 0.15);
	      servoRun = false; 
	      }
	    }
	}

	void init() {
		// Color Sensor Readings to Pin 0
		// Limit Switch to Pin 1, Pin 2
		i2c = new mraa::I2c(6);
		assert(i2c != NULL);
		//Turntable motor
		  dirTurn.dir(mraa::DIR_OUT);
		  dirTurn.write(0);
	}

	void readValues(){
		armVal = armLimit.read(); 
    	colorVal = colorSensor.read();
    	greenSwitch = limit1.read(); 
    	redSwitch = limit2.read();
	}
};

