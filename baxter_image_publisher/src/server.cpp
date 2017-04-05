#include <iostream>
#include <ros/ros.h>
#include <baxter_image_publisher/image_pub.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#define NODE_NAME "image_publisher_node"
#define SERVICE_NAME "image_publisher_service"
#define TOPIC_NAME "/robot/xdisplay"

using namespace cv;
using namespace std;


bool image_service(baxter_image_publisher::image_pub::Request &req,baxter_image_publisher::image_pub::Response &res)
{
	string image=req.name.data;
	ros::NodeHandle nh;

	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub=it.advertise("/robot/xdisplay",1);

	Mat frame=imread(image);

	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, frame).toImageMsg();

	for(int i=0;i<3;i++)
	{
		pub.publish(msg);
		sleep(1.0);
	}

	return true;
}


int main(int argc,char**argv)
{
	ros::init(argc,argv,NODE_NAME);
	ros::NodeHandle nh;

	ros::ServiceServer server=nh.advertiseService(SERVICE_NAME,image_service);

	cout<<"Image publisher service up . . . "<<endl;

	ros::spin();

	return 0;
}