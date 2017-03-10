#ifndef vision_hpp
#define vision_hpp

#include <opencv2/opencv.hpp>
#include "grip.hpp"

typedef double inch;
typedef double radian;



struct visionOutput {
	
	//if true, other values may be uninitalized.
	bool failure;
	
	// distance from the camera to the point directly in-between the tapes at the same height of the camera.
	const inch distance;
	
	//distance from the camera to the closest point directly in front of the tapes.
	const inch xDistance;
	
	//distance from the camera to the closest point on a horizontal line at the height of the camera crossing directly through the tapes.
	const inch yDistance;
	
	//the angle that the robot is from directly facing the tapes. (positive clockwise, negative counterclockwise)
	const radian robotAngle;
	
	const radian viewAngle;
	
	cv::Rect leftRect;
	cv::Rect rightRect;
};

visionOutput gearTarget(cv::Mat* image);

#endif /* vision_hpp */
