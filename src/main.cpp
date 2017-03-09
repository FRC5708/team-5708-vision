#include "vision.hpp"
#include <networktables/NetworkTable.h>
#include "misc.hpp"
#include <opencv2/opencv.hpp>
#include <unistd.h>
#include <thread>
#include <mutex>



/*
cv::Mat dummyImage() {
	cv::Mat image = cv::imread("/Users/max/Downloads/real rectangles.jpg");
	return image;
	//cv::Mat toReturn;
	//cv::cvtColor(image, toReturn, CV_BGRA2GRAY);
	//return toReturn;
	
}


void dump(visionOutput toDump) {
	printf("\nfailed: %s\n\
distance: %f\n\
x distance: %f\n\
y distance: %f\n\
robot angle: %f\n",
		   (toDump.failure? "true" : "false"), toDump.distance, toDump.xDistance, toDump.yDistance, toDump.robotAngle);
}

int main() {
	
	cv::Mat image = dummyImage();
	dump(gearTarget(&image));
}
*/

namespace vison5708Main {

	const int WORKER_THREAD_COUNT = 2;
	
	// frontCamera is 0, back is 1
	cv::VideoCapture frontCamera, backCamera;
	volatile bool usingFront = true, oneCamera = false;
	static std::shared_ptr<NetworkTable> table;
	// if oneCamera is true, backMat is empty
	cv::Mat frontMat, backMat;
	long frontMatTimestamp, backMatTimestamp;
	
	
	
	struct VisionThread {
		bool running;
		cv::Mat nextFrame;
		long nextFrameTimestamp;
		
		// probably would be better with semaphores
		std::mutex waitMutex;
	};
	VisionThread visionThreads[WORKER_THREAD_COUNT];
	std::mutex threadPoolMutex;
	
	inline std::chrono::milliseconds getTime() {
		return std::chrono::duration_cast<std::chrono::milliseconds>( std::chrono::steady_clock::now().time_since_epoch());
	}
	
	void changeCameras() {
		if (!oneCamera) usingFront = !usingFront;
	}
	
	void addJob(cv::Mat potentialJob, long timestamp) {
		threadPoolMutex.lock();
		
		int bestThread = 0;
		long bestTimeStamp = INT_MAX;
		for (int i = 0; i != WORKER_THREAD_COUNT; ++i) {
			if (!visionThreads[i].running) {
				bestThread = i;
				break;
			}
			if (visionThreads[i].nextFrameTimestamp < bestTimeStamp) {
				bestTimeStamp = visionThreads[i].nextFrameTimestamp;
				bestThread = i;
			}
		}
		
		visionThreads[bestThread].nextFrame = potentialJob;
		visionThreads[bestThread].nextFrameTimestamp = timestamp;
		if (!visionThreads[bestThread].running) visionThreads[bestThread].waitMutex.unlock();
	}
	
	void captureFrames() {
		while(true) {
			bool readFront = false, readBack = false;
			
			if (!oneCamera) {
				readFront = frontCamera.read(frontMat);
				if (!usingFront) readBack = backCamera.read(backMat);
			}
			else if (usingFront) readFront = frontCamera.read(frontMat);
			else readFront = backCamera.read(frontMat);
			
			long timestamp = getTime().count();
			
			
			if (!usingFront && readBack) {
				addJob(frontMat, timestamp);
			}
			
			
			// 5 milliseconds
			usleep(5000);
		}
	}
	
	
	void visionThreadRun(int id) {
		
		while(true) {
			
			threadPoolMutex.lock();
			
			visionThreads[id].running = true;
			cv::Mat workingData = visionThreads[id].nextFrame;
			long frameTimestamp = visionThreads[id].nextFrameTimestamp;
			
			threadPoolMutex.unlock();
			
			// this is a computation-heavy function, don't call it four times!
			visionOutput output = gearTarget(&workingData);
			
			table->PutNumber("xDist", output.xDistance);
			table->PutNumber("yDist", output.yDistance);
			table->PutNumber("angle", output.robotAngle);
			table->PutNumber("Dist", output.distance);
			
			threadPoolMutex.lock();
			
			visionThreads[id].running = false;
			
			// do we not have a new frame
			if (visionThreads[id].nextFrameTimestamp == frameTimestamp) {
				threadPoolMutex.unlock();
				
				// hack, don't do this
				
				visionThreads[id].waitMutex.lock();
				visionThreads[id].waitMutex.lock();
				visionThreads[id].waitMutex.unlock();
			}
			else {
				threadPoolMutex.unlock();
			}
		}
	}
	
	
	
	int run() {
		/*if (!backCamera.open(0)) oneCamera = true;
		
		if (!frontCamera.open(1) && !oneCamera) {
			oneCamera = true;
			usingFront = false;
		}
		else {
			printf("no cameras detected\n");
			return 1;
		}
		//networkClass.changeCameras = &changeCameras;*/
		frontCamera.open(0);
		oneCamera = true;
		usingFront = true;
		
		NetworkTable::SetClientMode();
		
		//this puts me on edge
		NetworkTable::SetIPAddresses({"10.57.8.21", "10.57.8.22", "10.57.8.23"});
		table = NetworkTable::GetTable("Vision");
		
		
		captureFrames();
		return 0;
	}
}

int main() {
	return vison5708Main::run();
}








