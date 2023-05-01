#include <stdio.h>
#include <iostream>
#include "opencv_aee.hpp"
#include "main.hpp"     // You can use this file for declaring defined values and functions
#include "pi2c.h"
#include<opencv2/opencv.hpp>
using namespace std;
using namespace cv;
void setup(void)
{
    setupCamera(320, 240);  // Enable the camera for OpenCV
    //Pi2c car(0x07); // Configure the I2C interface to the Car as a global variable
}
int SymbolRecognition(Mat frame)
{
    Mat framecopy = frame.clone(); // create a copy of the input frame
    cvtColor(frame, frame, COLOR_BGR2HSV);  //convert image to hsv

    // Define the color range to select pink objects in the image
    Scalar lower_range = Scalar(145,30,30);
    Scalar upper_range = Scalar(165,245,245);

    // Create a binary mask with the selected pink color range
    Mat pink_mask;
    inRange(frame, lower_range, upper_range, pink_mask);
    // Find the contour with the biggest area in the pink mask
    vector<vector<Point> > contours;//outer vector:multiple contours,inner vector: one contour,point structure:coordinate
    vector<Vec4i> hierarchy;//4 values,1:next contour in same hierarchy,2:previous contour at the same hierarchy,3:first child contour,4:parent contour

    findContours(pink_mask, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point());//retrieves all of the contours and reconstructs a full hierarchy of nested contours;compresses horizontal, vertical, and diagonal segments and leaves only their end points
    if(!contours.empty())
    {
        int savedContour = -1;
        double maxArea = 0.0;
        for (int i = 0; i< contours.size(); i++)//numbers of contours
        {
            double area = contourArea(contours[i]);
            if (area > maxArea)
            {
                maxArea = area;
                savedContour = i;
            }
        // Create a mask from the largest pink contour and draw it onto a copy of the input frame
        Mat mask = Mat::zeros(frame.size(), CV_8UC1);//type of the image as 8-bit single channel grayscale
        drawContours(mask, contours, savedContour, Scalar(255), FILLED, 8);
        Mat masked_image;
        frame.copyTo(masked_image, mask);
        // Find the contours of the pink mask to extract the outermost contour
        vector<vector<Point>> contours2;
        vector<Vec4i> hierarchy2;
        findContours(mask, contours2, hierarchy2, RETR_TREE, CHAIN_APPROX_SIMPLE);
        // Find the four corners of the outermost contour using approxPolyDP
        vector<Point> contour_poly;
        approxPolyDP(contours2[0], contour_poly, 3, true);
        vector<Point2i> corners;
        for (int i = 0; i < contour_poly.size(); i++)
        {
            corners.push_back(Point2i(contour_poly[i].x, contour_poly[i].y));
        }
        // Sort the corners in clockwise order, starting from the top left corner
        sort(corners.begin(), corners.end(), [](const Point2i& a, const Point2i& b) {return a.y < b.y;});
        vector<Point2i> corners_top = {corners[0], corners[1]};
        vector<Point2i> corners_bottom = {corners[2], corners[3]};
        sort(corners_top.begin(), corners_top.end(), [](const Point2i& a, const Point2i& b) {return a.x < b.x;});
        sort(corners_bottom.begin(), corners_bottom.end(), [](const Point2i& a, const Point2i& b) {return a.x > b.x;});
        corners = {corners_top[0], corners_top[1], corners_bottom[0], corners_bottom[1]};
        // Define the destination points for the transformation
        int xsize = 350;
        int ysize = 350;
        Point2f dst[4] = {Point2i(0, 0), Point2i(xsize, 0), Point2i(xsize, ysize), Point2i(0, ysize)};
        framecopy = transformPerspective(corners, framecopy, xsize, ysize);
        if (framecopy.cols > 0 && framecopy.rows > 0)
        {
            imshow("Output", framecopy);
            Mat Circle = imread("/home/pi/Desktop/symbol pics/Circle (Red Line).png");
            Mat Star = imread("/home/pi/Desktop/symbol pics/Star (color Line).png");
            Mat Triangle = imread("/home/pi/Desktop/symbol pics/Triangle (Blue Line).png");
            Mat Umbrella = imread("/home/pi/Desktop/symbol pics/Umbrella (Yellow Line).png");
            cvtColor (Circle, Circle, COLOR_BGR2GRAY);
            cvtColor (Star, Star, COLOR_BGR2GRAY);
            cvtColor (Triangle,Triangle, COLOR_BGR2GRAY);
            cvtColor (Umbrella, Umbrella, COLOR_BGR2GRAY);
            cvtColor (framecopy, framecopy, COLOR_BGR2GRAY);
			threshold(framecopy, framecopy, 128, 255, THRESH_BINARY); // apply binary threshold to grayscale image
			threshold(Circle, Circle, 128, 255, THRESH_BINARY);
			threshold(Star, Star, 128, 255, THRESH_BINARY);
			threshold(Triangle, Triangle, 128, 255, THRESH_BINARY);
			threshold(Umbrella, Umbrella, 128, 255, THRESH_BINARY);
            std::cout << "contour detected successfully" << std::endl;
            if(compareImages(framecopy,Circle)>80)
			{
				return 1;
                std::cout << "circle" << std::endl;
			}
            else if(compareImages(framecopy,Star)>80)
			{
				return 2;
                std::cout << "star" << std::endl;
			}
            else if(compareImages(framecopy,Triangle)>80)
			{
				return 3;
                std::cout << "triangle" << std::endl;
			}
            else if(compareImages(framecopy,Umbrella)>80)
			{
				return 4;
                std::cout << "umbrella" << std::endl;
			}
            else
			{
				return 0;
                std::cout << "no matchings found" << std::endl;
			}
        }
        else
		{
			return 0;
            std::cout << "contour size zero or negative" << std::endl;
		}
    else
	{
		return 0;
        std::cout << "contour is empty" << std::endl;
    }
    }
int main (int argc, char** argv)
{
	int det_color;
    setup();    // Call a setup function to prepare IO and devices
	namedWindow("Photo");   // Create a GUI window called photo
	int result=0;
    float error;
    float errorLast;
    float errorSum;
    int u;
    int Kp=15;
    int Ki=0;
    int Kd=0; //PID values
    Pi2c arduino(7); //sets up i2c, address 7 (also in arduino code)
	
	while(1)    // Main loop to perform image processing
    {
        Mat frame;
        while(frame.empty())
            frame = captureFrame(); // Capture a frame from the camera and store in a new matrix variable
        flip(frame,frame,-1);
        Mat color;
        cvtColor(frame,color,COLOR_BGR2HSV);
		vector<Mat> channels; // Array for channels
		split(frameHSV, channels); // Split the HSV into separate channels
		equalizeHist(channels[2], channels[2]); // Equalise the Value channel
		merge(channels, frameHSV); // Merge back into a single image
		if(result == 0)
		{
			// Define the color range to select pink objects in the image
			Scalar lower_range = Scalar(145,30,30);
			Scalar upper_range = Scalar(165,245,245);
			// Create a binary mask with the selected pink color range
			Mat pink_mask;
			inRange(color, lower_range, upper_range, pink_mask);
			int det_pink = countNonZero(pink_mask);
			if (det_pink > 80)
			{
				result = SymbolRecognition(frame);
			}
			inRange(color,Scalar(0,0,0), Scalar(179,255,51),color); //BLACK
		}
		Mat detect;
		int det_count;
		else if(result==1)
		{
			inRange(color,Scalar(0,50,50), Scalar(20,255,255),detect); //RED
			det_count=countNonZero(detect);
			if(det_count<100)
			{
				inRange(color,Scalar(0,0,0), Scalar(179,255,51),color); //BLACK
				result=0;
			}
			else
				inRange(color,Scalar(0,50,50), Scalar(20,255,255),color); //RED
		}
		else if(result==2)
		{
			inRange(color,Scalar(40,50,50), Scalar(80,255,255),detect); //GREEN
			det_count=countNonZero(detect);
			if(det_count<100)
			{
				inRange(color,Scalar(0,0,0), Scalar(179,255,51),color); //BLACK
				result=0;
			}
			else
				inRange(color,Scalar(40,50,50), Scalar(80,255,255),color); //GREEN
		}
		else if(result==3)
		{
			inRange(color,Scalar(100,50,50), Scalar(140,255,255),detect); //BLUE
			det_count=countNonZero(detect);
			if(det_count<100)
			{
				inRange(color,Scalar(0,0,0), Scalar(179,255,51),color); //BLACK
				result=0;
			}
			else
				inRange(color,Scalar(100,50,50), Scalar(140,255,255),color); //BLUE
		}
		else if(result==4)
		{
			inRange(color,Scalar(25,50,50), Scalar(35,255,255),detect); //YELLOW
			det_count=countNonZero(detect);
			if(det_count<100)
			{
				inRange(color,Scalar(0,0,0), Scalar(179,255,51),color); //BLACK
				result=0;
			}
			else
				inRange(color,Scalar(25,50,50), Scalar(35,255,255),color); //YELLOW
		}
		imshow("GreenLine",color);

        Mat L5p = color(Range(0,240),Range(0,32));
        imshow("L5",L5p);
        Mat L4p = color(Range(0,240),Range(32,64));
        imshow("L4",L4p);
        Mat L3p = color(Range(0,240),Range(64,96));
        imshow("L3",L3p);
        Mat L2p = color(Range(0,240),Range(96,128));
        imshow("L2",L2p);
        Mat L1p = color(Range(0,240),Range(128,160));
        imshow("L1",L1p);
        Mat R1p = color(Range(0,240),Range(160,192));
        imshow("R1",R1p);
        Mat R2p = color(Range(0,240),Range(192,224));
        imshow("R2",R2p);
        Mat R3p = color(Range(0,240),Range(224,256));
        imshow("R3",R3p);
        Mat R4p = color(Range(0,240),Range(256,288));
        imshow("R4",R4p);
        Mat R5p = color(Range(0,240),Range(288,320));
        imshow("R5",R5p);

        int l5w= -4.5;
        int l4w= -3.5;
        int l3w= -2.5;
        int l2w= -1.5;
        int l1w= -0.5;

        int r1w= 0.5;
        int r2w= 1.5;
        int r3w= 2.5;
        int r4w= 3.5;
        int r5w= 4.5;

        int l5n = countNonZero(L5p); //counts the NUMBER of white pixels
        int l4n = countNonZero(L4p);
        int l3n = countNonZero(L3p);
        int l2n = countNonZero(L2p);
        int l1n = countNonZero(L1p);

        int r1n = countNonZero(R1p);
        int r2n = countNonZero(R2p);
        int r3n = countNonZero(R3p);
        int r4n = countNonZero(R4p);
        int r5n = countNonZero(R5p);

        //if ((l5n && l4n && l3n && l2n &&l1n &&r1n && r2n && r3n &&r4n &&r5n) <100000)
        if ((l5n + l4n + l3n + l2n + l1n + r1n + r2n + r3n + r4n + r5n) <100)
        {
            u =99;
        }

        else
        {
            errorLast=error;
            error=((l5n*l5w+l4n*l4w+l3n*l3w+l2n*l2w+l1n*l1w+r5n*r5w+r4n*r4w+r3n*r3w+r2n*r2w+r1n*r1w)/(l5n+l4n+l3n+l2n+l1n+r5n+r4n+r3n+r2n+r1n+1));
            errorSum=errorSum+error;
            u=(Kp*error)+(Ki*errorSum)+(Kd*(error-errorLast));

        }

        arduino.i2cWriteArduinoInt(u);

        int key = cv::waitKey(1);   // Wait 1ms for a keypress (required to update windows)

        key = (key==255) ? -1 : key;    // Check if the ESC key has been pressed
        if (key == 27)
            break;
	}
	closeCV();  // Disable the camera and close any windows
	waitKey(0);
	return 0;
}