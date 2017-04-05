#include <iostream>
#include <ros/ros.h>
#include <baxter_image_publisher/image_pub.h>

#define NODE_NAME "image_publisher_client"
#define SERVICE_NAME "image_publisher_service"

using namespace std;


int main(int argc,char**argv)
{
	ros::init(argc,argv,NODE_NAME);
	ros::NodeHandle nh;

	ros::ServiceClient client=nh.serviceClient<baxter_image_publisher::image_pub>(SERVICE_NAME);

	baxter_image_publisher::image_pub srv;

	srv.request.name.data="/home/cptd/Downloads/Diag.png";

	if(client.call(srv))
	{
		cout<<"Success!"<<endl;
	}else
	{
		cout<<"An error has occured while starting the service!"<<endl;
	}
}
