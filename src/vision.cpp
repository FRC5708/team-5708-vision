#include "vision.hpp"
//#include <opencv.hpp>
#include <algorithm>
#include <math.h>
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
inch cameraHeight = 9.5; // need exact value
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
	
	std::vector<visionOutput> candidates;
	
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
				inch distance = (tapeHeight*tan(height)) / (1.0 - tapeBottomToCamera*cameraToTapeTop);
				//if camera is below tapes (never mind, doesn't work, above does, even with low cam) ??
				inch distance2 = (tapeHeight*tan(height)) / (1.0 + tapeBottomToCamera*cameraToTapeTop);
				
				printf("distances: %f %f\n", distance, distance2);
				
				inch xDistance = cos(width/2)*distance;
				inch yDistance = sin(width/2)*distance;
				
				//               midpoint of tapes                self-explainatory        center=0
				radian robotAngle = ((i.x + (j.x + j.width)) / 2.0 / (double)imageSize.width - 0.5) * 2.0 * cameraFOV;
				
				radian viewAngle = cos(yDistance/distance);
				
				
				double nanCheck = distance * xDistance * yDistance * robotAngle;
				if (!isnan(nanCheck) && !isinf(nanCheck) &&
					distance < 1.5 &&
						around(robotAngle, 0, cameraFOV/2)) {
					
					candidates.push_back({distance, xDistance, yDistance, robotAngle, viewAngle, i, j});
				}
			}
		}
	}
	// improve this later
	
	double bestSense = -1000;
	visionOutput* sensiestOutput = {0};
	
	FOREACH(candidates, i) {
		double senseScore = 0;
		
		radian heightDifference = i->leftRect.height - i->rightRect.height / (double)imageSize.height * cameraFOV;
		radian widthDifference = i->leftRect.width - i->rightRect.width / (double)imageSize.width * cameraFOV;
		
		senseScore -= heightDifference / tapeHeight + widthDifference /tapeWidth;
		
		
		if (senseScore > bestSense) {
			bestSense = senseScore;
			sensiestOutput = &*i;
		}
	}
	std::cout << sensiestOutput;
	if (sensiestOutput) {
		return *sensiestOutput;
	}
	
	debugRectDisplay(rects, imageSize);
	
	//dummy answer
	return {0,0,0,0};
}
