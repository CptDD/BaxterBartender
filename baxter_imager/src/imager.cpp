#include <ros/ros.h>
#include <iostream>
#include <baxter_core_msgs/DigitalIOState.h>
#include <opencv2/highgui/highgui.hpp>

#define NODE_NAME "baxter_imager"
#define TOPIC_NAME "/robot/digital_io/left_button_ok/state"

using namespace std;


void callback(const baxter_core_msgs::DigitalIOState::ConstPtr &msg)
{
	ros::NodeHandle nh;
	bool value;

	nh.getParam("/button_pressed",value);
	
	cout<<"Button value :"<<value<<endl;


	if(!value&&msg->state)
	{
		nh.setParam("/button_pressed",true);
		cout<<"The button has been pressed!"<<endl;
	}
}



int main(int argc,char**argv)
{
	ros::init(argc,argv,NODE_NAME);
	ros::NodeHandle nh;

	ros::Subscriber sub=nh.subscribe<baxter_core_msgs::DigitalIOState>(TOPIC_NAME,100,callback);

	nh.setParam("/button_pressed",false);

	ros::spin();

	return 0;
}