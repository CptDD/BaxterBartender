#include <iostream>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>


using namespace std;


int main(int argc,char**argv)
{
	ros::init(argc,argv,"transform_publisher");
	ros::NodeHandle nh;
	
	tf::TransformBroadcaster broadcaster;	
	tf::Transform transform,transform_2;
	tf::Quaternion q;
	q=tf::createQuaternionFromRPY(0,0,M_PI/4);
	transform.setOrigin(tf::Vector3(0.5,0.5,0));
	transform.setRotation(q);	

	transform_2.setRotation(q);
	double new_x=transform.getOrigin().x()+0.3
	double new_y=transform.getOrigin().y()-0.2;

	
	transform_2.setOrigin(tf::Vector3(new_x,new_y,transform.getOrigin().z()));
	
	while(ros::ok())
	{
		broadcaster.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"base","t2"));
		broadcaster.sendTransform(tf::StampedTransform(transform_2,ros::Time::now(),"base","t3"));
		sleep(0.2);
	}
	return 0;
}
