#include <iostream>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <baxter_current_pose/position.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>


using namespace std;

#define NODE_NAME "position_service"
#define SERVICE_NAME "position_service"

bool position_service(baxter_current_pose::position::Request &req,baxter_current_pose::position::Response &res)
{


	string arm=req.arm.data;
	string marker_id;

	cout<<"The arm is :"<<arm;

	//arm=="marker"?marker_id="/ar_marker_9":marker_id="/hole_target";

	marker_id=arm;	

	tf::TransformListener listener;
	tf::TransformBroadcaster broadcaster;

	tf::StampedTransform transform,new_t;
	tf::Transform transform_2;
	tf::Quaternion quat,new_quat;
	tf::Vector3 pos,new_pos;
	string error_msg;



	if(!listener.waitForTransform("/base",marker_id,ros::Time(0),ros::Duration(5.0),ros::Duration(0.01),&error_msg))
	{
		cout<<"Transform not ready! "<<error_msg<<endl;
	}else
	{
		try
		{
			listener.lookupTransform("/base",marker_id,ros::Time(0),transform);
			quat=transform.getRotation();
			pos=transform.getOrigin();


			/*double new_x=pos.x();
			double new_y=pos.y();

			cout<<"The x--y coordinates are :"<<new_x<<" "<<new_y<<endl;

			if(arm=="left")
			{
				new_x+=0.08;
				new_y+=0.09;
			}

			cout<<"The new x--y coordinates are :"<<new_x<<" "<<new_y<<endl;



			transform_2.setOrigin(tf::Vector3(new_x,new_y,transform.getOrigin().z()));
			transform_2.setRotation(quat);

			for(int i=0;i<10;i++)
			{
				broadcaster.sendTransform(tf::StampedTransform(transform_2,ros::Time::now(),marker_id,"marker_offset"));
				sleep(1.0);
			}



			new_pos=transform_2.getOrigin();
			new_quat=transform_2.getRotation();*/


		}catch(tf::TransformException ex)
		{
			cout<<"An exception has occured!==="<<ex.what()<<endl;
			return false;
		}
	}
	
	ros::spinOnce();

	geometry_msgs::Pose pose;
	pose.position.x=pos.x();
	pose.position.y=pos.y();
	pose.position.z=pos.z();

	pose.orientation.x=quat.getX();
	pose.orientation.x=quat.getY();
	pose.orientation.x=quat.getZ();
	pose.orientation.x=quat.getW();


	res.current_pose=pose;
	return true;
}


int main(int argc,char**argv)
{
	ros::init(argc,argv,NODE_NAME);
	ros::NodeHandle nh;	

	ros::ServiceServer service=nh.advertiseService(SERVICE_NAME,position_service);
	cout<<"Current pose server up and running . . ."<<endl;
	ros::spin();

	return 0;
}