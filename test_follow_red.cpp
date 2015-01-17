// Build with:
// // g++ test_follow_red.cpp -o test_follow_red `pkg-config opencv --cflags --libs` -lpthread -lmraa -std=c++11
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

#include <thread> //multithreading

//opencv libs

#include <sstream>
#include <string>
#include <iostream>
#include <stdio.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"

#define MS 1000

#define GYRO_DATA_OKAY_MASK 0x0C000000
#define GYRO_DATA_OKAY 0x04000000
#define PI 3.14159265


using namespace cv;

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

//
//opencv stuff
//

//initial min and max HSV filter values.
//these will be changed using trackbars
int H_MIN = 0;
int H_MAX = 256;
int S_MIN = 255;
int S_MAX = 256;
int V_MIN = 255;
int V_MAX = 256;
int GREEN_THRESHHOLD = 73;
int BLUE_THRESHHOLD = 50;
//default capture width and height
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;
//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS=50;
//minimum and maximum object area
const int MIN_OBJECT_AREA = 40*40;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;
//names that will appear at the top of each window

const int X_ZERO_POS = 320;
const int Y_ZERO_POS = 240;

const int X_ALPHA = .3;
int lastRedXPosition = 320;
int lastRedYPosition = 240;

float desiredAngle = 0.0;
float DEG_PER_PIXEL = 0.121875;

float total = 0; //current angle belief




string intToString(int number){


    std::stringstream ss;
    ss << number;
    return ss.str();
}

void drawObject(int x,int y,Mat &frame){

    cv::circle(frame,cv::Point(x,y),10,cv::Scalar(0,0,255));
    cv::putText(frame,intToString(x)+ " , " + intToString(y),cv::Point(x,y+20),1,1,Scalar(0,255,0));

}
void morphOps(Mat &thresh){

    //create structuring element that will be used to "dilate" and "erode" image.
    //the element chosen here is a 3px by 3px rectangle

    Mat erodeElement = getStructuringElement( MORPH_RECT,Size(3,3));
    //dilate with larger element so make sure object is nicely visible
    Mat dilateElement = getStructuringElement( MORPH_RECT,Size(8,8));

    erode(thresh,thresh,erodeElement);
    erode(thresh,thresh,erodeElement);


    dilate(thresh,thresh,dilateElement);
    dilate(thresh,thresh,dilateElement);



}
void trackFilteredObject(Mat threshold,Mat HSV, Mat &cameraFeed){

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
        //if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
        if(numObjects<MAX_NUM_OBJECTS){
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

                }else objectFound = false;


            }
            //let user know you found an object
            if(objectFound ==true){
                //draw object location on screen
                drawObject(x,y,cameraFeed);
                if (x!=0 && y!=0) {
                    lastRedXPosition = (lastRedXPosition*X_ALPHA) +  (x * (1 - X_ALPHA));
                    lastRedYPosition = (lastRedYPosition*X_ALPHA) +  (y * (1 - X_ALPHA));
                }
                printf("X: %i, Y: %i\n", lastRedXPosition, lastRedYPosition);
            }

        }else printf("Too much noise\n");;
    }
}

Mat& filterRed(Mat& I)
{
    // accept only char type matrices
    CV_Assert(I.depth() != sizeof(uchar));

    
    MatIterator_<Vec3b> it, end;
    int b;
    int g;
    int r;

    for( it = I.begin<Vec3b>(), end = I.end<Vec3b>(); it != end; ++it)
    {
        b = (*it)[0];
        g = (*it)[1];
        r = (*it)[2];
        if ( (r > g*(float(GREEN_THRESHHOLD)/50.0) ) && (r > b*(float(BLUE_THRESHHOLD)/50.0)) ){
            (*it)[0] = 0;
            (*it)[1] = 0;
            (*it)[2] = 255; 
        }
    }

    return I;
}

void cameraThreadLoop() {
    //
    //opencv stuff
    //

    //Matrix to store each frame of the webcam feed
    Mat cameraFeed;
    Mat threshold;
    Mat HSV;
    bool calibrationMode = true;

    float diffPixel = 0.0;

    //video capture object to acquire webcam feed
    VideoCapture capture;
    //open capture object at location zero (default location for webcam)
    capture.open(0);
    //set height and width of capture frame
    capture.set(CV_CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
    //start an infinite loop where webcam feed is copied to cameraFeed matrix
    //all of our operations will be performed within this loop

    while (running) {
        float currentAngle = total; 
        //store image to matrix
        capture.read(cameraFeed);
        // flip(cameraFeed,cameraFeed,1); //flip camera
        cameraFeed = filterRed(cameraFeed);
        //convert frame from BGR to HSV colorspace
        cvtColor(cameraFeed, HSV, COLOR_BGR2HSV);

        if (calibrationMode == true) {
            //if in calibration mode, we track objects based on the HSV slider values.
            cvtColor(cameraFeed, HSV, COLOR_BGR2HSV);
            inRange(HSV, Scalar(H_MIN, S_MIN, V_MIN), Scalar(H_MAX, S_MAX, V_MAX), threshold);
            morphOps(threshold);
            // imshow(windowName2,threshold);
            trackFilteredObject(threshold, HSV, cameraFeed);
        }


        diffPixel = float(lastRedXPosition - X_ZERO_POS);
        // integral += diffAngle * 0.001 * timeBetweenReadings;
        // derivative = (rf / 80.0);
        // power = speed * ((P_CONSTANT * diffAngle / 10000.0)); //+ (I_CONSTANT * integral) + (D_CONSTANT * derivative / 180.0)); //make sure to convert angles > 360 to proper angles
        float distanceToCube = 11.0 + lastRedYPosition*25.0;

        desiredAngle = currentAngle + (atan2(diffPixel,distanceToCube) * 180 / PI);
        printf("predicted Angle: %f\n", (atan2(diffPixel,distanceToCube) * 180 / PI));
        printf("Desired Angle: %f\n", desiredAngle);

        usleep(100 * MS);
    }
}

int main() {
    // Handle Ctrl-C quit
    signal(SIGINT, sig_handler);

    std::thread cameraThread(cameraThreadLoop);

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
    int init = 0;
    float rf;

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

    float speed = .05;
    // float desiredAngle = 0.0; //making this global so camera thread can use and change it
    float diffAngle = 0.0;
    float integral = 0;
    float power = 0;
    float derivative = 0;
    float timeBetweenReadings = 0;
    float gyroBias = 1.0;
    float forwardBias = 0;
    float P_CONSTANT = 45;
    float I_CONSTANT = 0;
    float D_CONSTANT = -1;

    

    while (running) {
        
        
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
            if (init) {
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
                init = 1;
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
        integral += diffAngle * 0.001 * timeBetweenReadings;
        derivative = (rf / 80.0);
        power = speed * ((P_CONSTANT * diffAngle / 360.0) + (I_CONSTANT * integral) + (D_CONSTANT * derivative / 180.0)); //make sure to convert angles > 360 to proper angles

        if (power > .3) {
            power = .3;
        } else if (power < -.3) {
            power = -.3;
        }
        setMotorSpeed(pwm, dir, -1 * power + forwardBias);
        setMotorSpeed(pwm2, dir2, -1 * power - forwardBias);
        printf("Set power to: %f\n", power);
        printf("Desired Angle: %f\n", desiredAngle);

    }

    return 0;
}
