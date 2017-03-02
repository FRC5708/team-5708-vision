#include "vision.hpp"
#include "misc.hpp"




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








