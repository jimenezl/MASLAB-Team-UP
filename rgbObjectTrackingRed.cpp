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

#include <sstream>
#include <string>
#include <iostream>
#include <stdio.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"

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
const string windowName1 = "Filtered Image";
const string windowName2 = "Thresholded Image";
const string windowName3 = "After Morphological Operations";
const string trackbarWindowName = "Trackbars";
void on_trackbar( int, void* )
{//This function gets called whenever a
	// trackbar position is changed





}
string intToString(int number){


	std::stringstream ss;
	ss << number;
	return ss.str();
}
void createTrackbars(){
	//create window for trackbars


	namedWindow(trackbarWindowName,0);
	//create memory to store trackbar name on window
	char TrackbarName[50];
	// sprintf( TrackbarName, "H_MIN", H_MIN);
	// sprintf( TrackbarName, "H_MAX", H_MAX);
	// sprintf( TrackbarName, "S_MIN", S_MIN);
	// sprintf( TrackbarName, "S_MAX", S_MAX);
	// sprintf( TrackbarName, "V_MIN", V_MIN);
	// sprintf( TrackbarName, "V_MAX", V_MAX);
	// sprintf( TrackbarName, "GREEN_THRESHHOLD", GREEN_THRESHHOLD);
	// sprintf( TrackbarName, "BLUE_THRESHHOLD", BLUE_THRESHHOLD);
	// sprintf( TrackbarName, "erodeElementSize", erodeElementSize);
	//create trackbars and insert them into window
	//3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
	//the max value the trackbar can move (eg. H_HIGH), 
	//and the function that is called whenever the trackbar is moved(eg. on_trackbar)
	//

	createTrackbar( "GREEN_THRESHHOLD", trackbarWindowName, &GREEN_THRESHHOLD, GREEN_THRESHHOLD, on_trackbar );
	createTrackbar( "BLUE_THRESHHOLD", trackbarWindowName, &BLUE_THRESHHOLD, BLUE_THRESHHOLD, on_trackbar );
	createTrackbar( "erodeElementSize", trackbarWindowName, &erodeElementSize, 20, on_trackbar );
	createTrackbar( "dilateElementSize", trackbarWindowName, &dilateElementSize, 20, on_trackbar );


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

void floodFilling(Mat *threshold, Mat *cameraFeed, int row, int col){

	// printf("at %d, %d\n", row, col);
	// std::cout << "value: " << threshold->at<bool>(row,col-1) << std::endl;
	threshold->at<uint8_t>(row,col) = 0; 
	cameraFeed->at<Vec3b>(row,col) = Vec3b(0,255,0);

	if ( ((col>1)&&(col<(FRAME_WIDTH-1))) && ((row>1)&&(row<(FRAME_HEIGHT-1))) ){
		if (threshold->at<bool>(row,col+1)){
			floodFilling(threshold,cameraFeed, row, col+1);
		}
		if (threshold->at<bool>(row,col-1)){
			floodFilling(threshold,cameraFeed, row, col-1);
		}
		if (threshold->at<bool>(row+1,col)){
			floodFilling(threshold,cameraFeed, row+1, col);
		}
		if (threshold->at<bool>(row-1,col)){
			floodFilling(threshold,cameraFeed, row-1, col);
		}
	}
}

void floodFillTracking(Mat *threshold, Mat *cameraFeed){
	printf("flood filling...\n");
	// std::cout << "depth: " << threshold->depth() << std::endl;
	// std::cout << "channel: " << threshold->channels() << std::endl;
	// std::cout << format(*threshold, "numpy") << std::endl;
	for (int row = 0; row < FRAME_HEIGHT; row=row+10){
		for (int col = 0; col < FRAME_WIDTH; col=col+10){
			// std::cout<<threshold->at<bool>(row,col) <<std::endl; //prints out 0 or 255
			bool value = threshold->at<bool>(row,col);
			if (value==true){
				floodFilling(threshold, cameraFeed, row, col);
				// cameraFeed->at<Vec3b>(row,col) = Vec3b(0,255,0);
				// threshold->at<uint8_t>(row,col) = 0;

			}
			
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

				}else objectFound = false;


			}
			//let user know you found an object
			if(objectFound ==true){
				//draw object location on screen
				drawObject(x,y,cameraFeed);}
	}
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
	//if we would like to calibrate our filter values, set to true.
	bool calibrationMode = true;
	
	//Matrix to store each frame of the webcam feed
	Mat cameraFeed;
	Mat threshold;
	Mat filteredImage;

	if(calibrationMode){
		//create slider bars for HSV filtering
		createTrackbars();
	}
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
		
		//convert frame from BGR to HSV colorspace
		// cvtColor(cameraFeed,HSV,COLOR_BGR2HSV);

		if(calibrationMode==true){
		//if in calibration mode, we track objects based on the HSV slider values.
		// cvtColor(cameraFeed,HSV,COLOR_BGR2HSV);
		cvtColor(filteredImage,threshold,CV_RGB2GRAY);
		// inRange(filteredImage,Scalar(254,254,254),Scalar(255,255,255),threshold);
		// morphOps(threshold);
		// blur(filteredImage, threshold, Size(10,10) );
		imshow(windowName2,threshold);
		// trackFilteredObject(threshold, cameraFeed);
		floodFillTracking(&threshold, &cameraFeed);
		}

		//show frames 
		imshow(windowName2,threshold);

		imshow(windowName,cameraFeed);
		imshow(windowName1,filteredImage);


		//delay 30ms so that screen can refresh.
		//image will not appear without this waitKey() command
		waitKey(30);
	}






	return 0;
}

