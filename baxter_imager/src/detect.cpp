#include <iostream>
#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/package.h>


using namespace std;
using namespace cv;



int main(int argc,char**argv)
{
	ros::init(argc,argv,"detect_node");

	string path=ros::package::getPath("baxter_imager");
	stringstream ss;
	ss<<path<<"/frames/Medium_up-2.jpg";


	Mat frame,frame_gray,binary;

	frame=imread(ss.str());

	cvtColor(frame,frame_gray,CV_BGR2GRAY);


	imshow("Example",frame);
	imshow("Gray",frame_gray);

	waitKey();
	return 0;
}