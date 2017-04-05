#include <iostream>
#include <ros/ros.h>
#include <baxter_detect_bulb/bulb_detect.h>
#include <baxter_core_msgs/EndEffectorState.h>

#define NODE_NAME "bulb_detector"
#define TOPIC_NAME "/robot/end_effector/left_gripper/state"
#define SERVICE_NAME "detect_service"

#define LARGE 50
#define MEDIUM 40


using namespace std;


double pos_value;

void callback(const baxter_core_msgs::EndEffectorState::ConstPtr &msg)
{
	pos_value=msg->position;
	ROS_INFO("Actual position value is :%g",pos_value);
}

bool detect_service(baxter_detect_bulb::bulb_detect::Request &req,baxter_detect_bulb::bulb_detect::Response &res)
{
	ros::NodeHandle nh;
	ros::Subscriber sub=nh.subscribe<baxter_core_msgs::EndEffectorState>(TOPIC_NAME,1,callback);

	for(int i=0;i<3;i++)
	{
		ros::spinOnce();
		sleep(1.0);
	}

	if(pos_value>=LARGE)
	{
		cout<<"Large!"<<endl;
		res.detection.data=0;
	}else if(pos_value>=MEDIUM && pos_value<LARGE)
	{
		cout<<"Medium!"<<endl;
		res.detection.data=1;
	}else
	{
		cout<<"Small!"<<endl;
		res.detection.data=2;
	}

	return true;
}


int main(int argc,char**argv)
{
	ros::init(argc,argv,NODE_NAME);	
	ros::NodeHandle nh;

	ros::ServiceServer server=nh.advertiseService(SERVICE_NAME,detect_service);

	ros::spin();

	return 0;
}