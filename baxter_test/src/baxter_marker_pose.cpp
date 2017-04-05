#include <iostream>
#include <ros/ros.h>
#include <tf/transform_listener.h>


using namespace std;



int main(int argc,char**argv)
{
	ros::init(argc,argv,"marker_current_position");
	ros::NodeHandle nh;

	
	tf::TransformListener listener;
	tf::StampedTransform marker_transform;

	string error_msg;
	tf::Quaternion marker_orientation;
	tf::Vector3 marker_position;


	if(!listener.waitForTransform("/base","/ar_marker_10",ros::Time(0),ros::Duration(5.0),ros::Duration(0.01),&error_msg))
	{
		cout<<"Transform not ready! "<<error_msg<<endl;
	}else
	{
		try
		{
			listener.lookupTransform("/base","/ar_marker_10",ros::Time(0),marker_transform);
			marker_orientation=marker_transform.getRotation();
			marker_position=marker_transform.getOrigin();

		}catch(tf::TransformException ex)
		{
			cout<<"An exception has occured!==="<<ex.what()<<endl;
		}
	}
	
	ros::spinOnce();

	cout<<"Transform Gripper ---> Marker"<<endl;
	cout<<"==Translation==\n"<<marker_position.getX()<<" "<<marker_position.getY()<<" "<<marker_position.getZ()<<"\n============="<<endl;
	cout<<"==Rotation==\n"<<marker_orientation.getX()<<" "<<marker_orientation.getY()<<" "<<marker_orientation.getZ()<<" "<<marker_orientation.getW()<<"\n================"<<endl;


	cout<<"Job finished!"<<endl;
	sleep(2.0);
	ros::shutdown();

	return 0;
}