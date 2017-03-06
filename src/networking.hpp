#ifndef networking_hpp
#define networking_hpp

#include "vision.hpp"



class RioNetworking {
	
public:
	// will return immediately. All timestamps in milliseconds
	void pushVisionOutput(visionOutput toPush, long timestamp);
	void pushFrame(cv::Mat frame);
	
	void (*changeCameras)(void);
};


#endif /* networking_hpp */
