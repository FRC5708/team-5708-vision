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

visionOutput visionFailure = {true, NAN, NAN, NAN, NAN, NAN};


template<typename T> inline bool around(T val1, T val2, T tolerance) {
	return (val1 > val2 - tolerance && val1 < val2 + tolerance);
}

//CONSTANTS
const inch cameraHeight = 9.5; // need exact value
const inch tapeBottomHeight = 10.75;
const inch tapeHeight = 5;
const inch tapeWidth = 2;
const inch tapeApart = 10.25; // from outside of tape
//radian cameraFOVHorizontal = (50/360)*2*M_PI; // may be approximations
//radian cameraFOVVertical = (30/60)*2*M_PI;
const radian cameraFOVHorizontal = (48.0/360.0)*2*M_PI;
const radian cameraFOVVertical = (36.0/360.0)*2*M_PI;

const inch groundToTapeTop = tapeBottomHeight + tapeHeight;
const inch tapeBottomToCamera = cameraHeight - tapeBottomHeight;
const inch cameraToTapeTop = groundToTapeTop - cameraHeight;


visionOutput computeFromBox(radian robotAngle, radian width, radian heightLeft, radian heightRight) {
	radian height = (heightLeft + heightRight)/2.0;
	
	double sqdis = sqrt(pow(tapeHeight, 2) + 4*tapeBottomToCamera*cameraToTapeTop*pow(tan(height), 2));
	inch distance = (tapeHeight + sqdis)/(2*tan(height));
	
	// law of sines
	double sineRatio = sin(width/2.0) / (tapeApart/2.0);
	radian viewAngle = M_PI - asin(sineRatio*distance) - width/2.0;
	
	inch xDistance = cos(viewAngle)*distance;
	inch yDistance = sin(viewAngle)*distance;
	
	radian straightViewAngle = M_PI/2.0 - viewAngle;
	// the best I could do
	if (heightLeft > heightRight){
		xDistance = -xDistance;
		straightViewAngle = -straightViewAngle;
	}
	
	
	double nanCheck = distance * xDistance * yDistance * robotAngle;
	if (!isnan(nanCheck) && !isinf(nanCheck) &&
		distance > 1.5 &&
		around(robotAngle, 0.0, cameraFOVHorizontal/2)) {
		
		return {false, distance, xDistance, yDistance, robotAngle, straightViewAngle};
	}
	else return visionFailure;
}


//todo: perspective distortion correction
visionOutput gearTarget(cv::Mat* image) {
	cv::Size imageSize = image->size();
	
	printf("processing image: width: %d height: %d\n", imageSize.width, imageSize.height);
	
	grip::ContourGrip finder;
	
	finder.Process(*image);
	std::vector<std::vector<cv::Point> >* contours = finder.GetFilterContoursOutput();
	
	std::vector<cv::Rect> rects(contours->size());
	
	FOREACH(*contours, i) {
		rects.push_back(cv::boundingRect(*i));
	}
	
	std::vector<visionOutput> candidates;
	
	
	
	const int rectWidthTolerance = .05*imageSize.width;
	const int rectHeightTolerance = .2*imageSize.height;
	const int minRectHeight = 60;
	
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
				
				radian heightLeft =  i.height / (double)imageSize.height * cameraFOVVertical;
				radian heightRight = j.height / (double)imageSize.height * cameraFOVVertical;
				
				//               midpoint of tapes                self-explainatory        center=0
				radian robotAngle = ((i.x + (j.x + j.width)) / 2.0 / (double)imageSize.width - 0.5) * 2.0 * cameraFOVHorizontal;

				visionOutput potential = computeFromBox(robotAngle, width, heightLeft, heightRight);
				if (!potential.failure) {
					potential.leftRect = i;
					potential.rightRect = j;
					candidates.push_back(potential);
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
	return visionFailure;
}



