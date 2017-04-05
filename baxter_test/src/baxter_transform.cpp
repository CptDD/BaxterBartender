#include <iostream>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <moveit/move_group_interface/move_group.h>

using namespace std;


/******
 * For light bulb tester: 0.03 on X, -0.11 on Y -- when right corner is used
 *                        0.07 on X, 0.03 on Y -- when left corner is used
 * For storage box: 0.12 on X, 0.04 on Y
 * Storage box next one :0.05 on X, 0.06 on Y
 ******/

//Current orientation :-0.0479944 0.997398 -0.00221891 0.0537557


int main(int argc,char**argv)
{
	ros::init(argc,argv,"pose_transformer");

	ros::NodeHandle nh;
	tf::TransformListener listener;
	tf::TransformBroadcaster broadcaster;

	moveit::planning_interface::MoveGroup group("left_arm");
	moveit::planning_interface::MoveGroup::Plan plan_right;


	ros::AsyncSpinner spinner(1);
	spinner.start();

	tf::StampedTransform transform,transform2;
	tf::Quaternion q,qq;
	tf::Vector3 p,pp;

	string error_msg;

	if(!listener.waitForTransform("/base","/ar_marker_9",ros::Time(0),ros::Duration(5.0),ros::Duration(0.01),&error_msg))
	{
		cout<<"Transform not ready! "<<error_msg<<endl;
	}else
	{
		try
		{
			listener.lookupTransform("/base","/ar_marker_9",ros::Time(0),transform);
			q=transform.getRotation();
			p=transform.getOrigin();

		}catch(tf::TransformException ex)
		{
			cout<<"An exception has occured!==="<<ex.what()<<endl;
		}
	}
	
	ros::spinOnce();

	cout<<"===Base - - - > Marker==="<<endl;
	cout<<"==Translation==\n"<<p.getX()<<" "<<p.getY()<<" "<<p.getZ()<<"\n============="<<endl;
	cout<<"==Rotation==\n"<<q.getX()<<" "<<q.getY()<<" "<<q.getZ()<<" "<<q.getW()<<"\n================"<<endl;
	cout<<"=============================="<<endl;

	if(!listener.waitForTransform("/right_gripper","/ar_marker_9",ros::Time(0),ros::Duration(5.0),ros::Duration(0.01),&error_msg))
	{
		cout<<"Transform not ready! "<<error_msg<<endl;
	}else
	{
		try
		{
			listener.lookupTransform("/right_gripper","/ar_marker_9",ros::Time(0),transform2);
			qq=transform2.getRotation();
			pp=transform2.getOrigin();

		}catch(tf::TransformException ex)
		{
			cout<<"An exception has occured!==="<<ex.what()<<endl;
		}
	}
	
	ros::spinOnce();

	cout<<"===Base - - - > Right Gripper==="<<endl;
	cout<<"==Translation==\n"<<pp.getX()<<" "<<pp.getY()<<" "<<pp.getZ()<<"\n============="<<endl;
	cout<<"==Rotation==\n"<<qq.getX()<<" "<<qq.getY()<<" "<<qq.getZ()<<" "<<qq.getW()<<"\n================"<<endl;
	cout<<"=============================="<<endl;




	/*
	tf::Transform test_transform;
	tf::Quaternion test_q;
	test_q=tf::createQuaternionFromRPY(0,0,0);



	cout<<"X  :"<<test_q.getX()<<" "<<test_q.getY()<<" "<<" "<<test_q.getZ()<<" "<<test_q.getW()<<endl;	

	test_transform.setOrigin(tf::Vector3(p.getX(),p.getY(),p.getZ()));
	test_transform.setRotation(test_q);
	//test_transform.setRotation(q);

	for(int i=0;i<10;i++)
	{
		broadcaster.sendTransform(tf::StampedTransform(test_transform,ros::Time::now(),"right_hand_camera","m_new_3"));
		cout<<"Transform sent!"<<endl;
		sleep(2.0);
	}*/


	group.setStartState(*group.getCurrentState());


	geometry_msgs::Pose target_pose;
	target_pose=group.getCurrentPose().pose;

	cout<<"Current position :"<<target_pose.position.x<<" "<<target_pose.position.y<<" "<<target_pose.position.z<<endl;
	cout<<"Current orientation :"<<target_pose.orientation.x<<" "<<target_pose.orientation.y<<" "<<target_pose.orientation.z<<" "
	<<target_pose.orientation.w<<endl;

	
	

    target_pose.position.x=p.getX()+0.09;
	target_pose.position.y=p.getY()+0.07;


	//target_pose.orientation.w=q.getW();


	group.setGoalPositionTolerance(1e-3);
	group.setGoalOrientationTolerance(1e-2);
	group.setStartState(*group.getCurrentState());
	group.setPoseTarget(target_pose);

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
	cout<<"Job finished!"<<endl;

	ros::shutdown();

	return 0;
}