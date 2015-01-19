//objectTrackingTutorial.cpp

//Written by  Kyle Hounslow 2013

//Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software")
//, to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, 
//and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

//The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
//IN THE SOFTWARE.
//compile with:
//g++ rgbObjectTrackingEdison.cpp -o rgbObjectTrackingEdison `pkg-config opencv --cflags --libs` -lpthread

#include <sstream>
#include <string>
#include <iostream>
#include <stdio.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include <unistd.h>

using namespace cv;
//initial min and max HSV filter values.
//these will be changed using trackbars
int H_MIN = 0;
int H_MAX = 256;
int S_MIN = 0;
int S_MAX = 256;
int V_MIN = 0;
int V_MAX = 256;
int GREEN_THRESHHOLD = 73;
int BLUE_THRESHHOLD = 44;
int erodeElementSize = 5;
int dilateElementSize = 13;

//default capture width and height
const int FRAME_WIDTH = 320;
const int FRAME_HEIGHT = 240;
//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS=50;
//minimum and maximum object area
const int MIN_OBJECT_AREA = 40*40;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;
//names that will appear at the top of each window
const string windowName = "Original Image";
const string windowName1 = "HSV Image";
const string windowName2 = "Thresholded Image";
const string windowName3 = "After Morphological Operations";
const string trackbarWindowName = "Trackbars";

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

	Mat erodeElement = getStructuringElement( MORPH_RECT,Size(erodeElementSize,erodeElementSize));
	//dilate with larger element so make sure object is nicely visible
	Mat dilateElement = getStructuringElement( MORPH_RECT,Size(dilateElementSize,dilateElementSize));

	erode(thresh,thresh,erodeElement);
	erode(thresh,thresh,erodeElement);


	dilate(thresh,thresh,dilateElement);
	dilate(thresh,thresh,dilateElement);



}
void trackFilteredObject(Mat threshold){

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
		// if(numObjects<MAX_NUM_OBJECTS){
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
				printf("object found at %d, %d\n", x, y);
			}

		// }else printf("Too much noise\n");
	}
}

void floodFilling(Mat *threshold, Mat *cameraFeed, int row, int col, long int & xTotal, long int & yTotal, long int & floodPixelCount){

	// printf("at %d, %d\n", row, col);
	// std::cout << "value: " << threshold->at<bool>(row,col-1) << std::endl;
	threshold->at<uint8_t>(row,col) = 0; 
	cameraFeed->at<Vec3b>(row,col) = Vec3b(0,255,0);
	xTotal +=col;
	yTotal +=row;
	floodPixelCount +=1;

	if ( ((col>1)&&(col<(FRAME_WIDTH-1))) && ((row>1)&&(row<(FRAME_HEIGHT-1))) ){
		if (threshold->at<bool>(row,col+1)){
			floodFilling(threshold,cameraFeed, row, col+1, xTotal, yTotal, floodPixelCount);
		}
		if (threshold->at<bool>(row,col-1)){
			floodFilling(threshold,cameraFeed, row, col-1, xTotal, yTotal, floodPixelCount);
		}
		if (threshold->at<bool>(row+1,col)){
			floodFilling(threshold,cameraFeed, row+1, col, xTotal, yTotal, floodPixelCount);
		}
		if (threshold->at<bool>(row-1,col)){
			floodFilling(threshold,cameraFeed, row-1, col, xTotal, yTotal, floodPixelCount);
		}
	}
}

void floodFillTracking(Mat *threshold, Mat *cameraFeed){
	printf("flood filling...\n");
	// std::cout << "depth: " << threshold->depth() << std::endl;
	// std::cout << "channel: " << threshold->channels() << std::endl;
	// std::cout << format(*threshold, "numpy") << std::endl;
	long int maxXCoord = 0;
	long int maxYCoord = 0;
	long int maxFloodPixelCount = 0;
	for (int row = 0; row < FRAME_HEIGHT; row=row+10){
		for (int col = 0; col < FRAME_WIDTH; col=col+10){
			// std::cout<<threshold->at<bool>(row,col) <<std::endl; //prints out 0 or 255
			bool value = threshold->at<bool>(row,col);
			long int xTotal = 0;
			long int yTotal = 0;
			long int floodPixelCount = 0;
			if (value==true){
				floodFilling(threshold, cameraFeed, row, col, xTotal, yTotal, floodPixelCount);
				// cameraFeed->at<Vec3b>(row,col) = Vec3b(0,255,0);
				// threshold->at<uint8_t>(row,col) = 0;

			}
			if (floodPixelCount>maxFloodPixelCount){
				maxXCoord = int(float(xTotal)/float(floodPixelCount));
				maxYCoord = int(float(yTotal)/float(floodPixelCount));
				maxFloodPixelCount = floodPixelCount;
			}
			
		}
	}
	printf("object found at %d, %d\n", maxXCoord, maxYCoord);

}

Mat& filterRed(Mat& filteredImage)
{
    // accept only char type matrices
    CV_Assert(filteredImage.depth() != sizeof(uchar));

    
    MatIterator_<Vec3b> it, end;
    int b;
    int g;
    int r;

    for( it = filteredImage.begin<Vec3b>(), end = filteredImage.end<Vec3b>(); it != end; ++it)
    {
    	b = (*it)[0];
    	g = (*it)[1];
    	r = (*it)[2];
    	if ( (r > g*(float(GREEN_THRESHHOLD)/50.0) ) && (r > b*(float(BLUE_THRESHHOLD)/50.0)) ){
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

int main(int argc, char* argv[])
{
	
	//Matrix to store each frame of the webcam feed
	Mat cameraFeed;
	Mat threshold;
	Mat filteredImage;

	int count = 0;

	//video capture object to acquire webcam feed
	VideoCapture capture;
	//open capture object at location zero (default location for webcam)
	capture.open(0);
	//set height and width of capture frame
	capture.set(CV_CAP_PROP_FRAME_WIDTH,FRAME_WIDTH);
	capture.set(CV_CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT);
	//start an infinite loop where webcam feed is copied to cameraFeed matrix
	//all of our operations will be performed within this loop
	while(1){
		//store image to matrix
		capture.read(cameraFeed);
		resize(cameraFeed, cameraFeed, Size(FRAME_WIDTH, FRAME_HEIGHT), 0, 0, INTER_CUBIC);
		// flip(cameraFeed,cameraFeed,1); //flip camera
		filteredImage = cameraFeed.clone();
		filteredImage = filterRed(filteredImage);
		
		cvtColor(filteredImage,threshold,CV_RGB2GRAY);
		floodFillTracking(&threshold, &cameraFeed);
		// morphOps(threshold);

		// trackFilteredObject(threshold);
		count += 1;
		printf("Processes One Frame: total %d\n", count);

		// usleep(20 * 1000);
	}






	return 0;
}

