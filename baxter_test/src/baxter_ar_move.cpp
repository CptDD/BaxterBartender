#include <iostream>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>

using namespace std;


int main(int argc,char**argv)
{
	ros::init(argc,argv,"current_pose");

	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	moveit::planning_interface::MoveGroup group("right_arm");
	moveit::planning_interface::MoveGroup::Plan plan_right;

	//group.setStartState(*group.getCurrentState());

	tf::TransformListener listener;
	tf::StampedTransform transform_marker;
	string error_msg;

	tf::Quaternion q;
	tf::Vector3 p;


	group.setStartStateToCurrentState();

	geometry_msgs::Pose pose=group.getCurrentPose().pose;

	cout<<"===Right gripper pose==="<<endl;
	cout<<"Position --->"<<endl;

	cout<<"X :"<<pose.position.x<<" Y:"<<pose.position.y<<" Z :"<<pose.position.z<<endl;

	cout<<"Orientation --->"<<endl;
	cout<<"X :"<<pose.orientation.x<<" Y :"<<pose.orientation.y<<" Z :"<<pose.orientation.z<<" W :"<<pose.orientation.w<<endl;
	cout<<"========================="<<endl;

	if(!listener.waitForTransform("/right_gripper","/ar_marker_10",ros::Time(0),ros::Duration(5.0),ros::Duration(0.01),&error_msg))
	{
		cout<<"Transform not ready! "<<error_msg<<endl;
	}else
	{
		try
		{
			listener.lookupTransform("/right_gripper","/ar_marker_10",ros::Time(0),transform_marker);
			q=transform_marker.getRotation();
			p=transform_marker.getOrigin();

		}catch(tf::TransformException ex)
		{
			cout<<"An exception has occured!==="<<ex.what()<<endl;
		}
	}
	
	ros::spinOnce();

	cout<<"===Right griper ---> Marker==="<<endl;
	cout<<"==Translation==\n"<<p.getX()<<" "<<p.getY()<<" "<<p.getZ()<<"\n============="<<endl;
	cout<<"==Rotation==\n"<<q.getX()<<" "<<q.getY()<<" "<<q.getZ()<<" "<<q.getW()<<"\n================"<<endl;
	cout<<"=============================="<<endl;

	cout<<"New X should be :"<<p.getX()+pose.position.x<<endl;
	cout<<"New Y should be :"<<p.getY()+pose.position.y<<endl;


	//pose.position.x+=p.getX();
	pose.position.x+=p.getX();
	pose.position.y+=p.getY();
	//pose.position.y=p_g.getY()+p_m.getY();

	group.setGoalPositionTolerance(1e-3);
	group.setGoalOrientationTolerance(1e-2);
	group.setStartState(*group.getCurrentState());
	group.setPoseTarget(pose);


	cout<<"Trying to find a plan!"<<endl;


	if(group.plan(plan_right))
	{
		cout<<"Found a plan moving the arm!"<<endl;
		sleep(3.0);
		group.move();
	}else
	{
		cout<<"Plan has not been found!"<<endl;
	}

	sleep(2.0);

	cout<<"Reading new pose ..."<<endl;

	pose=group.getCurrentPose().pose;
	cout<<"===Right gripper pose==="<<endl;
	cout<<"Position --->"<<endl;

	cout<<"X :"<<pose.position.x<<" Y:"<<pose.position.y<<" Z :"<<pose.position.z<<endl;

	cout<<"Orientation --->"<<endl;
	cout<<"X :"<<pose.orientation.x<<" Y :"<<pose.orientation.y<<" Z :"<<pose.orientation.z<<" W :"<<pose.orientation.w<<endl;
	cout<<"========================="<<endl;	


	cout<<"Job finished!"<<endl;
	sleep(2.0);
	return 0;
}