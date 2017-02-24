#ifndef vision_hpp
#define vision_hpp

typedef double inch;
typedef double radian;



struct visionOutput {
	
	// distance from the camera to the point directly in-between the tapes at the same height of the camera.
	const inch distance;
	
	//distance from the camera to the closest point directly in front of the tapes.
	const inch xDistance;
	
	//distance from the camera to the closest point on a horizontal line at the height of the camera crossing directly through the tapes.
	const inch yDistance;
	
	//the angle that the robot is from directly facing the tapes.
	const radian robotAngle;
};

visionOutput gearTarget();

#endif /* vision_hpp */
