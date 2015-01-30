//objectTrackingTutorial.cpp
//Compile on PC:
//g++ -ggdb `pkg-config --cflags opencv` -o `basename rgbObjectTrackingEdisonMultithread.cpp` rgbObjectTrackingEdisonMultithread `pkg-config --libs opencv`
//Compile on Edison:
//g++ robot_state_machine.cpp -o robot_state_machine `pkg-config opencv --cflags --libs` -lpthread -lmraa -std=c++11


#include <sstream>
#include <string>
#include <iostream>
#include <stdio.h>
// #include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"

#include <unistd.h>
#include <thread> //multithreading

#include <stdint.h>
#include <signal.h>
#include <sys/time.h>

#include <cassert>
#include <cmath>
#include <csignal>
#include <math.h>

#include "mraa.hpp"
#define SHIELD_I2C_ADDR 0x40
#define MS 1000

using namespace cv;


int ROBOT_STATE = 0; //0 is wall following, 1 is block tracking, 2 is block picking up, 3 is sorting blocks


//initial min and max HSV filter values.
//these will be changed using trackbars
float GREEN_THRESHHOLD_FOR_RED = 73.0/50.0;
float BLUE_THRESHHOLD_FOR_RED = 44.0/50.0;
float GREEN_THRESHHOLD_FOR_BLUE = 47.0/50.0;
float RED_THRESHHOLD_FOR_BLUE = 100.0/50.0;
float RED_THRESHHOLD_FOR_GREEN = 59.0/50.0;
float BLUE_THRESHHOLD_FOR_GREEN = 28.0/50.0;
float CYAN_THRESHHOLD_FOR_YELLOW = 55.0/50.0;
float MAGENTA_THRESHHOLD_FOR_YELLOW = 46.0/50.0;


long int thresholdBlockSize = 12000; //Number of pixels needed for a cube to be considered "close enough" to be picked up
float stackedTwoBlockThreshhold = 1.2; //min amount that the vertical stack has to be greater than the horizontal for a block to be counted as a stack
float stackedThreeBlockThreshhold = 1.8; //min amount that the vertical stack has to be greater than the horizontal for a block to be counted as a stack
int minimumBlockSize = 20*20; //min number of pixels to be considered a block


//default capture width and height
const int FRAME_WIDTH = 320;
const int FRAME_HEIGHT = 240;
//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS=50;
//minimum and maximum object area
const int MIN_OBJECT_AREA = 40*40;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;

float objectAngle = 0.0;
float ANGLE_ALPHA = .3;
float distanceToBlock = 0;

int loopsInPickupRange = 0;

bool blockFound = false;

float desiredAngle = 0.0;
float DEGREES_PER_PIXEL = 0.14;

float total = 0; //current angle belief

int initGyro = 0;

//wall follower constants
float DISTANCE_FROM_IR_SENSORS = 4.6;
int BACK_INFRARED_PIN = 3;
int FRONT_INFRARED_PIN = 2;
int HEAD_INFRARED_PIN = 1;

float alpha_infrareds = .3;


//motor stuff
int running = 1;

//picking up blocks
mraa::I2c *i2c;

mraa::Gpio dirTurn = mraa::Gpio(3); //Direction of Turntable
mraa::Gpio dirArm = mraa::Gpio(4); //Direction of Arm

void sig_handler(int signo) {
    if (signo == SIGINT) {
        printf("closing spi nicely\n");
        running = 0;
        mraa::Pwm pwm = mraa::Pwm(9);
        mraa::Pwm pwm2 = mraa::Pwm(6);
        pwm.write(0);
        pwm2.write(0);
     //    setMotorPosition(11, 0.0);
	    // setServoPosition(7, 1.6);
	    // setServoPosition(0, -0.20);
	    // setMotorPosition(8, 0);
    }
}

void setServoPosition(int index, double duty);

void setMotorPosition(int index, double duty);

long timevalDiff(struct timeval *starttime, struct timeval *finishtime) { //returns time difference
  long msec;
  msec=(finishtime->tv_sec-starttime->tv_sec);
  //msec+=(finishtime->tv_usec-starttime->tv_usec)/1000000;
  return msec;
}

//
// Wall Follower Stuff
//

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

//
// Motor Stuff
//

void setMotorSpeed(mraa::Pwm &pwm, mraa::Gpio &dir, double speed) {
    assert(-1.0 <= speed && speed <= 1.0);
    if (speed < 0) {
        dir.write(1);
    } else {
        dir.write(0);
    }
    pwm.write(fabs(speed));
}

//sorting blocks

float cvalOne = 0;
float colorVal = 0;

float greenSwitch = 0;
float redSwitch = 0; 
bool servoRun = true;


// Forward declarations of checkColors limitSwitches
void checkColors(float colorVal);
void limitSwitches(float switch1, float switch2, bool servoRun);

// Limit Switches
void limitSwitches(float switch1, float switch2, bool servoRun){

  if (switch1 < 1) {
    printf("Turning off motor\n");
    setMotorPosition(8, 0.0);
    sleep(2.0); // give time for motor to turn off

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
  else if (switch2 < 1){
    printf("Turning off motor\n");
    setMotorPosition(8, 0.0);
    sleep(2.0);
    
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
// no block less than a480 
// red min is 630
// green min is 480 
// red > 620
// green 

// Check color sensors and move to hopper
// green 450 to 565
// 370 < no block > 420
//  565 > red
void checkColors(float colorVal){
  if (610 < colorVal) { 
      printf("Red Block Found\n");
      servoRun = true;
      dirTurn.write(0);

      // adding in check for already being at red station
      if (redSwitch < 1){
        limitSwitches(redSwitch, greenSwitch, servoRun);
      }
      else {
        printf("Turntable moving\n");
        setMotorPosition(8, 0.12);
        sleep(2.0); // Give time for turntable to get to new pos.
        limitSwitches(greenSwitch, redSwitch, servoRun);
      }
    }
  else if (colorVal < 525){  
      printf("Green Block Found\n");
      servoRun = true;
      dirTurn.write(1);
       // adding in check for already being at green station
      if (greenSwitch < 1){
        limitSwitches(greenSwitch, redSwitch, servoRun);
      }
      else { 
        printf("Turntable moving\n");
        setMotorPosition(8, 0.12);
        sleep(2.0); // Give time for turntable to get to new pos.
        limitSwitches(greenSwitch, redSwitch, servoRun);
      }
    }
  /*else if (430 < colorVal){
      printf("No Block Found\n"); //prev > 1000
      dirTurn.write(1);
      // adding in check for already being at green station
      if (greenSwitch < 1){
        limitSwitches(greenSwitch, redSwitch, servoRun);
      }
      else { 
        printf("Turntable moving\n");
        setMotorPosition(8, 0.12);
        sleep(2.0); // Give time for turntable to get to new pos.
        limitSwitches(greenSwitch, redSwitch, servoRun);
    }
  } 
  */
}




// Motor Setup
uint8_t registers[] = {
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


//
// Camera Stuff
//

void on_trackbar( int, void* ) {
	//This function gets called whenever a
	// trackbar position is changed
}

string intToString(int number){
	std::stringstream ss;
	ss << number;
	return ss.str();
}

//Comment out this method if not using GUI
// void createTrackbars(){
// 	//create window for trackbars
// 	namedWindow(trackbarWindowName,0);
// 	//create memory to store trackbar name on window
// 	char TrackbarName[50];

// 	createTrackbar( "GREEN_THRESHHOLD_FOR_RED", trackbarWindowName, &GREEN_THRESHHOLD_FOR_RED, GREEN_THRESHHOLD_FOR_RED, on_trackbar );
// 	createTrackbar( "BLUE_THRESHHOLD_FOR_RED", trackbarWindowName, &BLUE_THRESHHOLD_FOR_RED, BLUE_THRESHHOLD_FOR_RED, on_trackbar );
// 	createTrackbar( "erodeElementSize", trackbarWindowName, &erodeElementSize, 20, on_trackbar );
// 	createTrackbar( "dilateElementSize", trackbarWindowName, &dilateElementSize, 20, on_trackbar );
// }

void drawObject(int x,int y,Mat &frame){

	cv::circle(frame,cv::Point(x,y),10,cv::Scalar(0,0,255));
	cv::putText(frame,intToString(x)+ " , " + intToString(y),cv::Point(x,y+20),1,1,Scalar(0,255,0));

}
// void morphOps(Mat &thresh){

// 	//create structuring element that will be used to "dilate" and "erode" image.
// 	//the element chosen here is a 3px by 3px rectangle

// 	Mat erodeElement = getStructuringElement( MORPH_RECT,Size(erodeElementSize,erodeElementSize));
// 	//dilate with larger element so make sure object is nicely visible
// 	Mat dilateElement = getStructuringElement( MORPH_RECT,Size(dilateElementSize,dilateElementSize));

// 	erode(thresh,thresh,erodeElement);
// 	erode(thresh,thresh,erodeElement);

// 	dilate(thresh,thresh,dilateElement);
// 	dilate(thresh,thresh,dilateElement);
// }

void floodFillingWalls(Mat *threshold, Mat *cameraFeed, int row, int col, long int & xTotal, long int & yTotal, long int & floodPixelCount, int & minX, int & minY, int & maxX, int & maxY){

	// printf("at %d, %d\n", row, col);
	// std::cout << "value: " << threshold->at<bool>(row,col-1) << std::endl;
	
	cameraFeed->at<Vec3b>(row,col) = Vec3b(0,255,0);
	if (threshold->at<bool>(row,col)){
		for (int i = 0; i <= row; i++){
			cameraFeed->at<Vec3b>(i,col) = Vec3b(0,0,0);
			// if(!cameraFeed->at<bool>(i+1,col)){
			// 	break;
			// }
		}
	}

	threshold->at<uint8_t>(row,col) = 0; 

	xTotal +=col;
	yTotal +=row;
	floodPixelCount +=1;

	if (minX+minY > col+row){
		minX = col;
		minY = row;
	}
	if (maxX+maxY < col+row){
		maxX = col;
		maxY = row;
	}

	if ( ((col>1)&&(col<(FRAME_WIDTH-1))) && ((row>1)&&(row<(FRAME_HEIGHT-1))) ){
		if (threshold->at<bool>(row,col+1)){
			floodFillingWalls(threshold,cameraFeed, row, col+1, xTotal, yTotal, floodPixelCount, minX, minY, maxX, maxY);
		}
		if (threshold->at<bool>(row,col-1)){
			floodFillingWalls(threshold,cameraFeed, row, col-1, xTotal, yTotal, floodPixelCount, minX, minY, maxX, maxY);
		}
		if (threshold->at<bool>(row+1,col)){
			floodFillingWalls(threshold,cameraFeed, row+1, col, xTotal, yTotal, floodPixelCount, minX, minY, maxX, maxY);
		}
		if (threshold->at<bool>(row-1,col)){
			floodFillingWalls(threshold,cameraFeed, row-1, col, xTotal, yTotal, floodPixelCount, minX, minY, maxX, maxY);
		}
	}
}

void floodFillTrackingWalls(Mat *threshold, Mat *cameraFeed){
	// printf("flood filling...\n");
	// std::cout << "depth: " << threshold->depth() << std::endl;
	// std::cout << "channel: " << threshold->channels() << std::endl;
	// std::cout << format(*threshold, "numpy") << std::endl;
	long int objectXCoord = 0;
	long int objectYCoord = 0;
	int objectMaxX = 0;
	int objectMaxY = 0;
	int objectMinX = 0;
	int objectMinY = 0;

	long int maxFloodPixelCount = 0;

	for (int row = 0; row < FRAME_HEIGHT; row=row+10){
		for (int col = 0; col < FRAME_WIDTH; col=col+10){
			// std::cout<<threshold->at<bool>(row,col) <<std::endl; //prints out 0 or 255
			bool value = threshold->at<bool>(row,col);
			long int xTotal = 0;
			long int yTotal = 0;
			long int floodPixelCount = 0;
			int maxX = 0;
			int maxY = 0;
			int minX = 320;
			int minY = 240;

			if (value==true){
				floodFillingWalls(threshold, cameraFeed, row, col, xTotal, yTotal, floodPixelCount, minX, minY, maxX, maxY);
				// cameraFeed->at<Vec3b>(row,col) = Vec3b(0,255,0);
				// threshold->at<uint8_t>(row,col) = 0;
			}

			// if (floodPixelCount>maxFloodPixelCount){
			// 	objectXCoord = int(float(xTotal)/float(floodPixelCount));
			// 	objectYCoord = int(float(yTotal)/float(floodPixelCount));
			// 	maxFloodPixelCount = floodPixelCount;

			// 	objectMaxX = maxX;
			// 	objectMaxY = maxY;
			// 	objectMinX = minX;
			// 	objectMinY = minY;
			// }
			
		}
	}
	
	// printf("object found at %d, %d, pixel count: %d\n", objectXCoord, objectYCoord, maxFloodPixelCount);
	// printf("border coordinates: (%d,%d) , (%d,%d)\n", objectMinX, objectMinY, objectMaxX, objectMaxY);
	// drawObject(objectMinX, objectMinY,*cameraFeed);
	// drawObject(objectMaxX, objectMaxY,*cameraFeed);
	
	// drawObject(objectXCoord,objectYCoord,*cameraFeed);
}

void floodFilling(Mat *threshold, Mat *cameraFeed, int row, int col, long int & xTotal, long int & yTotal, long int & floodPixelCount, int & minX, int & minY, int & maxX, int & maxY){

	// printf("at %d, %d\n", row, col);
	// std::cout << "value: " << threshold->at<bool>(row,col-1) << std::endl;
	threshold->at<uint8_t>(row,col) = 0; 
	cameraFeed->at<Vec3b>(row,col) = Vec3b(0,255,0);
	xTotal +=col;
	yTotal +=row;
	floodPixelCount +=1;

	if (minX+minY > col+row){
		minX = col;
		minY = row;
	}
	if (maxX+maxY < col+row){
		maxX = col;
		maxY = row;
	}

	if ( ((col>1)&&(col<(FRAME_WIDTH-1))) && ((row>1)&&(row<(FRAME_HEIGHT-1))) ){
		if (threshold->at<bool>(row,col+1)){
			floodFilling(threshold,cameraFeed, row, col+1, xTotal, yTotal, floodPixelCount, minX, minY, maxX, maxY);
		}
		if (threshold->at<bool>(row,col-1)){
			floodFilling(threshold,cameraFeed, row, col-1, xTotal, yTotal, floodPixelCount, minX, minY, maxX, maxY);
		}
		if (threshold->at<bool>(row+1,col)){
			floodFilling(threshold,cameraFeed, row+1, col, xTotal, yTotal, floodPixelCount, minX, minY, maxX, maxY);
		}
		if (threshold->at<bool>(row-1,col)){
			floodFilling(threshold,cameraFeed, row-1, col, xTotal, yTotal, floodPixelCount, minX, minY, maxX, maxY);
		}
	}
}

void floodFillTracking(Mat *threshold, Mat *cameraFeed){
	// printf("flood filling...\n");
	// std::cout << "depth: " << threshold->depth() << std::endl;
	// std::cout << "channel: " << threshold->channels() << std::endl;
	// std::cout << format(*threshold, "numpy") << std::endl;
	long int objectXCoord = 320;
	long int objectYCoord = 240;
	int objectMaxX = 0;
	int objectMaxY = 0;
	int objectMinX = 0;
	int objectMinY = 0;

	long int maxFloodPixelCount = 0;

	for (int row = 0; row < FRAME_HEIGHT; row=row+10){
		for (int col = 0; col < FRAME_WIDTH; col=col+10){
			// std::cout<<threshold->at<bool>(row,col) <<std::endl; //prints out 0 or 255
			bool value = threshold->at<bool>(row,col);
			long int xTotal = 0;
			long int yTotal = 0;
			long int floodPixelCount = 0;
			int maxX = 0;
			int maxY = 0;
			int minX = 320;
			int minY = 240;

			if (value==true){
				floodFilling(threshold, cameraFeed, row, col, xTotal, yTotal, floodPixelCount, minX, minY, maxX, maxY);
				// cameraFeed->at<Vec3b>(row,col) = Vec3b(0,255,0);
				// threshold->at<uint8_t>(row,col) = 0;

			}
			if (floodPixelCount>maxFloodPixelCount){
				objectXCoord = int(float(xTotal)/float(floodPixelCount));
				objectYCoord = int(float(yTotal)/float(floodPixelCount));
				maxFloodPixelCount = floodPixelCount;

				objectMaxX = maxX;
				objectMaxY = maxY;
				objectMinX = minX;
				objectMinY = minY;
			}
			
			
		}
	}
	
	printf("object found at %d, %d, pixel count: %d\n", objectXCoord, objectYCoord, maxFloodPixelCount);
	printf("border coordinates: (%d,%d) , (%d,%d)\n", objectMinX, objectMinY, objectMaxX, objectMaxY);
	printf("object angle: %f\n", float(objectXCoord-160) * 0.2125);
	// objectAngle = float(objectXCoord-160) * 0.2125;
	// 
	if (maxFloodPixelCount > minimumBlockSize){
		objectAngle = (objectAngle*ANGLE_ALPHA) +  (float(objectXCoord-160) * DEGREES_PER_PIXEL * (1 - ANGLE_ALPHA));
		// drawObject(objectMinX, objectMinY,*cameraFeed);
		// drawObject(objectMaxX, objectMaxY,*cameraFeed);

		int numOfBlocks = 1;
		if (float(objectMaxX-objectMinX)*stackedThreeBlockThreshhold<(objectMaxY- objectMinY)){
			// drawObject(objectXCoord, objectYCoord+int((objectMaxY-objectMinY)*.33),*cameraFeed);
			// drawObject(objectXCoord, objectYCoord,*cameraFeed);
			// drawObject(objectXCoord, objectYCoord-int((objectMaxY-objectMinY)*.33),*cameraFeed);
			numOfBlocks = 3;
		}
		else if (float(objectMaxX-objectMinX)*stackedTwoBlockThreshhold<(objectMaxY- objectMinY)){
			// drawObject(objectXCoord, objectYCoord+int((objectMaxY-objectMinY)*.25),*cameraFeed);
			// drawObject(objectXCoord, objectYCoord-int((objectMaxY-objectMinY)*.25),*cameraFeed);
			numOfBlocks = 2;
		}
		else {
			// drawObject(objectXCoord,objectYCoord,*cameraFeed);
		}

		// distanceToBlock = 1 - (float(numOfBlocks*maxFloodPixelCount) / float(thresholdBlockSize));
		distanceToBlock = 170 - objectMaxY;
		blockFound = true;
		if (distanceToBlock<25 && fabs(objectAngle) < 5){
	        	loopsInPickupRange++;
		}
	}
	else {
		blockFound = false;
		loopsInPickupRange = 0;
		if (ROBOT_STATE = 1){
			ROBOT_STATE = 0;
		}
	}
}



void trackFilteredObject(Mat threshold, Mat &cameraFeed){

	int x,y;

	Mat temp;
	threshold.copyTo(temp);
	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );
	//use moments method to find our filtered object
	double refArea = 0;
	bool objectFound = false;
	if (hierarchy.size() > 0) {
		int numObjects = hierarchy.size();
			for (int index = 0; index >= 0; index = hierarchy[index][0]) {

				Moments moment = moments((cv::Mat)contours[index]);
				double area = moment.m00;

				//if the area is less than 20 px by 20px then it is probably just noise
				//if the area is the same as the 3/2 of the image size, probably just a bad filter
				//we only want the object with the largest area so we safe a reference area each
				//iteration and compare it to the area in the next iteration.
				if(area>MIN_OBJECT_AREA){
					x = moment.m10/area;
					y = moment.m01/area;

					objectFound = true;

				} else objectFound = false;


			}
			//let user know you found an object
			if(objectFound ==true){
				//draw object location on screen
				drawObject(x,y,cameraFeed);}
	}
}

Mat& filterBlue(Mat& filteredImage)
{
    // accept only char type matrices
    // CV_Assert(filteredImage.depth() != sizeof(uchar));

    
    MatIterator_<Vec3b> it, end;
    int b;
    int g;
    int r;

    int y;
    int m;
    int c;

    for( it = filteredImage.begin<Vec3b>(), end = filteredImage.end<Vec3b>(); it != end; ++it)
    {
    	b = (*it)[0];
    	g = (*it)[1];
    	r = (*it)[2];

    	y = r+g;
    	c = g+b;
    	m = r+b;

    	if ( (b > g*GREEN_THRESHHOLD_FOR_BLUE ) && (b > r*RED_THRESHHOLD_FOR_BLUE) ){
    		(*it)[0] = 255;
	        (*it)[1] = 255;
	        (*it)[2] = 255;	
    	}
    	// else if ( (y > c*CYAN_THRESHHOLD_FOR_YELLOW ) && (y > m*MAGENTA_THRESHHOLD_FOR_YELLOW) ){
    	// 	(*it)[0] = 255;
	    //     (*it)[1] = 255;
	    //     (*it)[2] = 255;	
    	// }
    	else {
    		(*it)[0] = 0;
	        (*it)[1] = 0;
	        (*it)[2] = 0;	
    	}
    }

    return filteredImage;
}

Mat& filterBlock(Mat& filteredImage)
{
    // accept only char type matrices
    // CV_Assert(filteredImage.depth() != sizeof(uchar));

    
    MatIterator_<Vec3b> it, end;
    int b;
    int g;
    int r;

    for( it = filteredImage.begin<Vec3b>(), end = filteredImage.end<Vec3b>(); it != end; ++it)
    {
    	b = (*it)[0];
    	g = (*it)[1];
    	r = (*it)[2];
    	if ( (r > g*GREEN_THRESHHOLD_FOR_RED) && (r > b*BLUE_THRESHHOLD_FOR_RED) ) {
    		(*it)[0] = 255;
	        (*it)[1] = 255;
	        (*it)[2] = 255;	
    	}
    	else if ( (g > r*RED_THRESHHOLD_FOR_GREEN ) && (g > b*BLUE_THRESHHOLD_FOR_GREEN) ){
    		(*it)[0] = 255;
	        (*it)[1] = 255;
	        (*it)[2] = 255;	
    	}
    	else {
    		(*it)[0] = 0;
	        (*it)[1] = 0;
	        (*it)[2] = 0;	
    	}
    }

    return filteredImage;
}

VideoCapture capture;
Mat currentImage;
// std::mutex currentImageMutex;

void cameraBufferLoop() {
	while(true){
		// currentImageMutex.lock();
		capture.read(currentImage);
		// currentImageMutex.unlock();
	}
}

void cameraThreadLoop() {
	long int frameCount = 0; 
	//Matrix to store each frame of the webcam feed
	Mat cameraFeed;
	Mat threshold;
	Mat filteredImage;

	float diffPixel = 0.0;
	//comment out if not using GUI
	// createTrackbars();


	//video capture object to acquire webcam feed
	// VideoCapture capture;
	//open capture object at location zero (default location for webcam)
	capture.open(0);
	//set height and width of capture frame
	capture.set(CV_CAP_PROP_FRAME_WIDTH,FRAME_WIDTH);
	capture.set(CV_CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT);
	//start an infinite loop where webcam feed is copied to cameraFeed matrix
	//all of our operations will be performed within this loop
	capture.read(currentImage);
	std::thread cameraBufferThread(cameraBufferLoop);
	while(1){
		float currentAngle = total; 

		//store image to matrix
		capture.read(cameraFeed);
		resize(cameraFeed, cameraFeed, Size(FRAME_WIDTH, FRAME_HEIGHT), 0, 0, INTER_CUBIC);
		// flip(cameraFeed,cameraFeed,1); //flip camera

		filteredImage = cameraFeed.clone();

		filteredImage = filterBlue(filteredImage);

		cvtColor(filteredImage,threshold,CV_RGB2GRAY);

		floodFillTrackingWalls(&threshold, &cameraFeed);

		filteredImage = cameraFeed.clone();

		filteredImage = filterBlock(filteredImage);

		cvtColor(filteredImage,threshold,CV_RGB2GRAY);

		floodFillTracking(&threshold, &cameraFeed);

		frameCount +=1;
		printf("Frames processed: %d\n", frameCount);

		// diffPixel = float(lastRedXPosition - X_ZERO_POS);
        // integral += diffAngle * 0.001 * timeBetweenReadings;
        // derivative = (rf / 80.0);
        // power = speed * ((P_CONSTANT * diffAngle / 10000.0)); //+ (I_CONSTANT * integral) + (D_CONSTANT * derivative / 180.0)); //make sure to convert angles > 360 to proper angles

        desiredAngle = currentAngle + objectAngle;
        printf("predicted Angle: %f\n", desiredAngle - currentAngle);
        printf("Desired Angle: %f\n", desiredAngle);

        usleep(100 * MS);

        if (ROBOT_STATE == 0){
			if (blockFound){
				ROBOT_STATE = 1;
				initGyro = 0; //set this to 0 to reset the gyro
			} else {
				usleep(400 * MS);
			}        	
        }
	}
}

int main() {
    // Handle Ctrl-C quit
    signal(SIGINT, sig_handler);

    struct timeval startTime;
    gettimeofday(&startTime, NULL);

    struct timeval currentTime;
    gettimeofday(&currentTime, NULL);

    std::thread cameraThread(cameraThreadLoop);

    //Picking up blocks init stuff

    mraa::Gpio armLimit = mraa::Gpio(2); // Arm Limit Switch

	// Edison i2c bus is 6
	i2c = new mraa::I2c(6);
	assert(i2c != NULL);

	// Arm motor
	dirArm.dir(mraa::DIR_OUT);
	dirArm.write(0); // Arm going down

	initPWM();

	mraa::Pwm pwm = mraa::Pwm(9);
	pwm.write(0.0);
	pwm.enable(true);
	//assert(pwm != NULL);
	mraa::Gpio dir = mraa::Gpio(8);
	//assert(dir != NULL);
	dir.dir(mraa::DIR_OUT);
	dir.write(1);

	mraa::Pwm pwm2 = mraa::Pwm(6);
	pwm2.write(0.0);
	pwm2.enable(true);
	//assert(pwm2 != NULL);
	mraa::Gpio dir2 = mraa::Gpio(5);
	//assert(dir != NULL);
	dir2.dir(mraa::DIR_OUT);
	dir2.write(1);

    //gyro stuff
    mraa::Gpio *chipSelect = new mraa::Gpio(10);
    chipSelect->dir(mraa::DIR_OUT);
    chipSelect->write(1);
    mraa::Spi *spi = new mraa::Spi(0);
    spi->bitPerWord(32);
    char rxBuf[2];
    char writeBuf[4];
    unsigned int sensorRead = 0x20000000;
    writeBuf[0] = sensorRead & 0xff;
    writeBuf[1] = (sensorRead >> 8) & 0xff;
    writeBuf[2] = (sensorRead >> 16) & 0xff;
    writeBuf[3] = (sensorRead >> 24) & 0xff;
    // float total = 0; made this global for camera thread
    struct timeval tv;
    
    float rf;

    //wall following ir's
    mraa::Aio aioBackInfrared = mraa::Aio(BACK_INFRARED_PIN);
    mraa::Aio aioFrontInfrared = mraa::Aio(FRONT_INFRARED_PIN);
    mraa::Aio aioHeadInfrared = mraa::Aio(HEAD_INFRARED_PIN);

    float speed = .2;
    // float desiredAngle = 0.0; //making this global so camera thread can use and change it
    float diffAngle = 0.0;
    float integral = 0;
    float power = 0;
    float derivative = 0;
    float timeBetweenReadings = 0;
    float gyroBias = 1.0;
    float forwardBias = 0.0;
    float P_CONSTANT = 75;
    float I_CONSTANT = .1;
    float D_CONSTANT = -1;

    float previousDistance = 0;

    float P_CONSTANT_DISTANCE = .00175;
    float D_CONSTANT_DISTANCE = .0004;

    

    //wall follower constants
    float speedWallFollower = .2;
    float powerWallFollower = 0;
    float forwardBiasWallFollower = .25;

    float backDistance = 0;
    float frontDistance = 0;
    float headDistance = 10.0;

    float desiredDistance = 7;

    float P_CONSTANT_WALL_FOLLOWER = .2;

    while (running) {
    	printf("Robot state: %d\n", ROBOT_STATE);

    	gettimeofday(&currentTime, NULL);

    	if (timevalDiff(&startTime, &currentTime) > 180){
    		break;
    	}


    	if (ROBOT_STATE == 0) {

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

	        if (powerWallFollower > .4) {
	            powerWallFollower = .4;
	        } else if (powerWallFollower < -.4) {
	            powerWallFollower = -.4;
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

	    } else if (ROBOT_STATE == 1){
	        
	        
	        //gyro stuff 
	        chipSelect->write(0);
	        char *recv = spi->write(writeBuf, 4);
	        chipSelect->write(1);
	        //    printf("%x %x %x %x\r\n", recv[0], recv[1], recv[2], recv[3]);
	        if (recv != NULL) {
	            unsigned int recvVal = ((uint8_t) recv[3] & 0xFF);
	            recvVal = (recvVal << 8) | ((uint8_t)recv[2] & 0xFF);
	            recvVal = (recvVal << 8) | ((uint8_t)recv[1] & 0xFF);
	            recvVal = (recvVal << 8) | ((uint8_t)recv[0] & 0xFF);
	            // printf("Received: 0x%.8x, ", recvVal);
	            // Sensor reading
	            short reading = (recvVal >> 10) & 0xffff;
	            if (initGyro) {
	                unsigned long long ms = (unsigned long long)(tv.tv_sec) * 1000 +
	                                        (unsigned long long)(tv.tv_usec) / 1000;
	                gettimeofday(&tv, NULL);
	                ms -= (unsigned long long)(tv.tv_sec) * 1000 +
	                      (unsigned long long)(tv.tv_usec) / 1000;
	                int msi = (int)ms;
	                float msf = (float)msi;
	                timeBetweenReadings = -msf;
	                rf = (float)reading;
	                total += -0.001 * msf * ((rf / 80.0) + gyroBias); // -(rf/80.0) is angular rate (deg/sec)
	                printf("Total: %f\n", total);
	            } else {
	                initGyro = 1;
	                gettimeofday(&tv, NULL);
	            }
	        } else {
	            printf("No recv\n");
	        }
	        usleep(10 * MS);

	        //Fix angle readings over 360
	        if (total > 360) {
	            int error = total/360;
	            total = total - error*360;
	        }

	        else if (total < -360) {
	            int error = fabs(total)/360;
	            total = total + 360*error;
	        }



	        diffAngle = desiredAngle - total;
	        integral = integral*.9 + (diffAngle * 0.001 * timeBetweenReadings);
	        derivative = (rf / 80.0);
	        power = speed * ((P_CONSTANT * diffAngle / 360.0) + (I_CONSTANT * integral) + (D_CONSTANT * derivative / 180.0)); //make sure to convert angles > 360 to proper angles
	        
	        if (fabs(diffAngle)<5){
		        forwardBias = (P_CONSTANT_DISTANCE * distanceToBlock) + (D_CONSTANT_DISTANCE * (distanceToBlock - previousDistance));
		        printf("D term: %f\n", (D_CONSTANT_DISTANCE * (distanceToBlock - previousDistance)));
		        printf("Distance: %d\n", distanceToBlock);
		        // forwardBias = 0.0;
		    } else {
		    	forwardBias = 0.0;
		    }

	        if (power > .3) {
	            power = .3;
	        } else if (power < -.3) {
	            power = -.3;
	        }

	        previousDistance = distanceToBlock;

	        
	        if (loopsInPickupRange > 15){
	        	printf("picking up blocks\n");
	        	ROBOT_STATE = 2;
	        	// break;
	        }
	        printf("loopsInPickupRange: %d\n", loopsInPickupRange);
	        
	        setMotorSpeed(pwm, dir, -1 * power + forwardBias);
	        setMotorSpeed(pwm2, dir2, -1 * power - forwardBias);
	        printf("Set power to: %f\n", power);
	        printf("Desired Angle: %f\n", desiredAngle);

		} else if (ROBOT_STATE == 2) {
			mraa::Gpio armLimit = mraa::Gpio(2); // Arm Limit Switch

			// Arm motor
			dirArm.dir(mraa::DIR_OUT);
			dirArm.write(0); // Arm going down

			// Arm Dropping and Claw Opening
			setServoPosition(7, 1.6);
			usleep(1000*400);
			printf("Dropping arm\n");
			setMotorPosition(11, 0.3);
			usleep(1000*1400);
			setMotorPosition(11, 0.0);
			printf("sleeping\n");
			sleep(2.0);
			setServoPosition(0, -0.2);
			sleep(2.0);

			// Move forwared 0.5 seconds
			float speedPickup = .172; 
			printf("Moving Forward\n");
			setMotorSpeed(pwm, dir, speedPickup);
			setMotorSpeed(pwm2, dir2, -1 * speedPickup);
			usleep(1000*2500);
			setMotorSpeed(pwm, dir, 0);
			setMotorSpeed(pwm2, dir2, 0);
			sleep(2.0);

			dirArm.write(1); // Arm going up
			printf("Claw closing\n");
			setServoPosition(0, 0.3); //Claw Closing
			sleep(1.0);

			int armVal = 1;

			while(armVal > 0){ 
			armVal = armLimit.read();// Arm moving up until switch hit
			setMotorPosition(11, 0.25);
			printf("Arm Going up\n");
			}

			printf("Switch Hit!!\n");
			setMotorPosition(11, 0.0); ///turn off motor
			sleep(1.0);
			setServoPosition(7,1.1); // close gate
			printf("Arm Limit: %d\n", armVal);
			dirArm.write(0); 
			usleep(1000*300);

			printf("Dropping blocks\n");
			setServoPosition(0, 0.0); //open claw up partially 
			sleep(2.0); 

			//starting to sort

			// Color Sensor Readings to Pin 0
			// Limit Switch to Pin 1, Pin 2
			mraa::Aio colorSensor = mraa::Aio(0);
			mraa::Gpio limit1 = mraa::Gpio(1); 
			mraa::Gpio limit2 = mraa::Gpio(0);  

			float count = 0;

			//Turntable motor
			dirTurn.dir(mraa::DIR_OUT);
			dirTurn.write(0);

			initPWM();

			// Sort blocks by color
			while(count < 4){
			colorVal = colorSensor.read();
			greenSwitch = limit1.read(); 
			redSwitch = limit2.read();
			std::cout << "Colors: " << colorVal << std::endl;
			std::cout << "Green Switch: " << greenSwitch << std::endl;
			std::cout << "Red Switch: " << redSwitch << std::endl;  
			checkColors(colorVal); //checking color sensor
			count+=1;
			sleep(3.0);
			}
			count = 0;
			while (greenSwitch > 0){
			dirTurn.write(1);
			setMotorPosition(8, .12);
			sleep(1.0);
			}
			ROBOT_STATE = 0;
		}
    }

    setMotorSpeed(pwm, dir, 0);
    setMotorSpeed(pwm2, dir2, 0);
    return 0;
}

