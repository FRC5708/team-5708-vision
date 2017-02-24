#include "vision.hpp"
//#include <opencv.hpp>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <math.h>
#include "contours processing.hpp"
#include "misc.hpp"
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/core.hpp>


//does not return
void debugDisplayMat(cv::Mat toDisplay) {
	cv::imshow("hi", toDisplay);
	cv::waitKey();
}

cv::Mat dummyImage() {
	cv::Mat image = cv::imread("/Users/max/Downloads/real rectangles.jpg");
	return image;
	//cv::Mat toReturn;
	//cv::cvtColor(image, toReturn, CV_BGRA2GRAY);
	//return toReturn;
	
}

#define straightLineTolerance (50) //slope
#define minLineLength (10) //pixels



void debugRectDisplay(std::vector<cv::Rect> rects, cv::Size size) {
	cv::Mat drawing = cv::Mat::zeros(size, CV_8UC3);
	
	FOREACH(rects, i) {
		rectangle( drawing, i->tl(), i->br(), cvScalar( 100, 255, 100 ), 2, 8, 0 );
	}
	debugDisplayMat(drawing);
}

int rectSizeTolerance = 30;
int minRectHeight = 60;

inline bool around(int val1, int val2, int tolerance) {
	return (val1 > val2 - tolerance && val1 < val2 + tolerance);
}

//CONSTANTS
inch cameraHeight = 8; // need exact value
inch tapeBottomHeight = 10.75;
inch tapeHeight = 5;
inch tapeWidth = 2;
inch tapeApart = 10.25; // from outside of tape
radian cameraFOV = 0.25*2*M_PI; // need exact value

inch groundToTapeTop = tapeBottomHeight + tapeHeight;
inch tapeBottomToCamera = cameraHeight - tapeBottomHeight;
inch cameraToTapeTop = groundToTapeTop - cameraHeight;


//todo: perspective distortion correction
visionOutput gearTarget() {
	grip::ContourGrip finder;
	
	cv::Mat image = dummyImage();
	cv::Size imageSize = image.size();
	finder.Process(image);
	std::vector<std::vector<cv::Point> >* contours = finder.GetFilterContoursOutput();
	
	std::vector<cv::Rect> rects(contours->size());
	
	FOREACH(*contours, i) {
		rects.push_back(cv::boundingRect(*i));
	}
	
	// loop through all rects.
	for (auto iP = rects.begin(); iP != rects.end(); ++iP) {
		for (auto jP = rects.begin(); jP != rects.end(); ++jP) {
			cv::Rect i = *iP;
			cv::Rect j = *jP;
			if (iP != jP &&
				around(i.width, j.width, rectSizeTolerance) &&
				around(i.height, j.height, rectSizeTolerance) &&
				(i.x + i.width) < j.x &&
				i.height >= minRectHeight && j.height >= minRectHeight) {
				
				
				radian width = (j.x + j.width - i.x) / (double)imageSize.width * cameraFOV;
				radian height = (i.height + j.height) / 2.0 / (double)imageSize.height * cameraFOV;
				
				// if camera height is within tape bounds
				inch distance = (groundToTapeTop*tan(height)) / (1.0 - tapeBottomToCamera*cameraToTapeTop);
				//if camera is below tapes (never mind, doesn't work, above does, even with low cam)
				inch distance2 = (tapeHeight*tan(height)) / (1.0 + tapeBottomToCamera*cameraToTapeTop);
				
				printf("distances: %f %f\n", distance, distance2);
				
				//radian viewAngle = M_PI - width - asin(sin(width)*(tapeApart/2.0)*distance);
				
				//printf("angle: %f\n", viewAngle);
			}
		}
	}
	// filter out nonsensical answers (todo)
	
	
	debugRectDisplay(rects, imageSize);
	
	//dummy answer
	return {0,0,0,0};
}
