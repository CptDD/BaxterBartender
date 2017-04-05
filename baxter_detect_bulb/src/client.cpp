#include <iostream>
#include <ros/ros.h>
#include <baxter_detect_bulb/bulb_detect.h>

using namespace std;

#define NODE_NAME "detect_client"
#define SERVICE_NAME "detect_service"

int main(int argc,char**argv)
{
	ros::init(argc,argv,NODE_NAME);
	ros::NodeHandle nh;

	ros::ServiceClient client=nh.serviceClient<baxter_detect_bulb::bulb_detect>(SERVICE_NAME);

	baxter_detect_bulb::bulb_detect srv;


	if(client.call(srv))
	{
		cout<<"Success!"<<endl;
		cout<<srv.response.detection.data<<endl;

	}else
	{
		cout<<"Not success!"<<endl;
	}

	return 0;
}

