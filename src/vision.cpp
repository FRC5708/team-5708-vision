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
void debugRectDisplay(std::vector<cv::Rect> rects, cv::Size size) {
	cv::Mat drawing = cv::Mat::zeros(size, CV_8UC3);
	
	FOREACH(rects, i) {
		rectangle( drawing, i->tl(), i->br(), cvScalar( 100, 255, 100 ), 2, 8, 0 );
	}
	debugDisplayMat(drawing);
}


#define straightLineTolerance (50) //slope
#define minLineLength (10) //pixels




template<typename T> inline bool around(T val1, T val2, T tolerance) {
	return (val1 > val2 - tolerance && val1 < val2 + tolerance);
}

//CONSTANTS
inch cameraHeight = 9.5; // need exact value
inch tapeBottomHeight = 10.75;
inch tapeHeight = 5;
inch tapeWidth = 2;
inch tapeApart = 10.25; // from outside of tape
//radian cameraFOVHorizontal = (50/360)*2*M_PI; // may be approximations
//radian cameraFOVVertical = (30/60)*2*M_PI;
radian cameraFOVHorizontal = ((60/sqrt(337)*16)/360)*2*M_PI;
radian cameraFOVVertical = ((60/sqrt(337)*9)/360)*2*M_PI;

inch groundToTapeTop = tapeBottomHeight + tapeHeight;
inch tapeBottomToCamera = cameraHeight - tapeBottomHeight;
inch cameraToTapeTop = groundToTapeTop - cameraHeight;


//todo: perspective distortion correction
visionOutput gearTarget(cv::Mat* image) {
	cv::Size imageSize = image->size();
	
	grip::ContourGrip finder;
	
	finder.Process(*image);
	std::vector<std::vector<cv::Point> >* contours = finder.GetFilterContoursOutput();
	
	std::vector<cv::Rect> rects(contours->size());
	
	FOREACH(*contours, i) {
		rects.push_back(cv::boundingRect(*i));
	}
	
	std::vector<visionOutput> candidates;
	
	
	
	int rectWidthTolerance = .05*imageSize.width;
	int rectHeightTolerance = .2*imageSize.height;
	int minRectHeight = 60;
	
	// loop through all rects.
	for (auto iP = rects.begin(); iP != rects.end(); ++iP) {
		for (auto jP = rects.begin(); jP != rects.end(); ++jP) {
			cv::Rect i = *iP;
			cv::Rect j = *jP;
			if (iP != jP &&
				i.height >= minRectHeight && j.height >= minRectHeight &&
				around(i.width, j.width, rectWidthTolerance) &&
				around(i.height, j.height, rectHeightTolerance) &&
				(i.x + i.width) < j.x) {
				
				
				radian width = (j.x + j.width - i.x) / (double)imageSize.width * cameraFOVHorizontal;
				radian height = (i.height + j.height) / 2.0 / (double)imageSize.height * cameraFOVVertical;
				
				// if camera height is within tape bounds
				//inch distance = (tapeHeight*tan(height)) / (1.0 - tapeBottomToCamera*cameraToTapeTop);
				//if camera is below tapes (never mind, doesn't work, above does, even with low cam) ??
				//inch distance2 = (tapeHeight*tan(height)) / (1.0 + tapeBottomToCamera*cameraToTapeTop);
				
				//double b = tapeHeight / tan(height);
				//double sqdis = sqrt(b*b + 4*tapeBottomHeight*cameraToTapeTop);
				double tanh = tan(height);
				double sqdis = sqrt(pow(tapeHeight, 2) + 4*tapeBottomToCamera*cameraToTapeTop*pow(tanh, 2));
				
				
				double distance = (tapeHeight + sqdis)/(2*tanh);
				// I don't think this is nessecary, but never know
				//double distance2 = (tapeHeight - sqdis)/(2*tanh);
				
				radian viewAngle = cos(tapeApart / 2 / distance);
				
				inch xDistance = cos(viewAngle/2.0)*distance;
				inch yDistance = sin(viewAngle/2.0)*distance;
				
				//               midpoint of tapes                self-explainatory        center=0
				radian robotAngle = ((i.x + (j.x + j.width)) / 2.0 / (double)imageSize.width - 0.5) * 2.0 * cameraFOVHorizontal;
				
				double nanCheck = distance * xDistance * yDistance * robotAngle;
				if (!isnan(nanCheck) && !isinf(nanCheck) &&
					distance > 1.5 &&
						around(robotAngle, 0.0, cameraFOVHorizontal/2)) {
					
					candidates.push_back({false, distance, xDistance, yDistance, robotAngle, i, j});
				}
			}
		}
	}
	// improve this later
	
	double bestSense = -1000;
	visionOutput* sensiestOutput = {0};
	
	FOREACH(candidates, i) {
		double senseScore = 0;
		
		radian heightDifference = i->leftRect.height - i->rightRect.height / (double)imageSize.height * cameraFOVVertical;
		radian widthDifference = i->leftRect.width - i->rightRect.width / (double)imageSize.width * cameraFOVHorizontal;
		
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
	
	// failure value
	return {true};
}


