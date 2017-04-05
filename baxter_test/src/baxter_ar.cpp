#include <iostream>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <moveit/move_group_interface/move_group.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>


using namespace std;

#define NODE_DESCRIPTION "A node used to place the end effector to a predefined position based on the detection of an AR tag"
#define NODE_NAME "baxter_ar"

#define RIGHT_ARM_PLANNING_GROUP "right_arm"
#define LEFT_ARM_PLANNING_GROUP "left_arm"
#define BOTH_ARMS_PLANNING_GROUP "both_arms"

#define KNOWN_POSITION_X  0.71
#define KNOWN_POSITION_Y -0.18

#define MARKER_POSITION_X 0.74
#define MARKER_POSITION_Y -0.11
#define MARKER_POSITION_Z 0.775


/*int main(int argc,char**argv)
{
	ros::init(argc,argv,NODE_NAME);

	ros::NodeHandle nh;
	tf::TransformListener listener;
	tf::StampedTransform transform;

	try
	{
		listener.lookupTransform("right_hand_camera","ar_marker_9",ros::Time::now(),transform);
	}catch(tf::TransformException ex)
	{
		cout<<"An exception has occured!"<<endl;
		cout<<ex.what()<<endl;
	}


	cout<<NODE_DESCRIPTION<<endl;

	return 0;
}*/


void callback(const ar_track_alvar_msgs::AlvarMarkers &msg)
{
	cout<<"Message has arrived!"<<endl;
	geometry_msgs::Pose pose=msg.markers[0].pose.pose;

	cout<<pose.position<<endl;

}	


int main(int argc,char**argv)
{
	ros::init(argc,argv,NODE_NAME);
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(1);

	moveit::planning_interface::MoveGroup group_right(RIGHT_ARM_PLANNING_GROUP);
	moveit::planning_interface::MoveGroup::Plan plan_right;

	spinner.start();

	tf::TransformListener listener;
	tf::StampedTransform marker_transform,gripper_transform,transform;
	string error_msg;
	tf::Quaternion q_m,q_g,q;
	tf::Vector3 p_m,p_g,p;

	if(!listener.waitForTransform("/right_gripper","/ar_marker_10",ros::Time(0),ros::Duration(5.0),ros::Duration(0.01),&error_msg))
	{
		cout<<"Transform not ready! "<<error_msg<<endl;
	}else
	{
		try
		{
			listener.lookupTransform("/right_gripper","/ar_marker_10",ros::Time(0),marker_transform);
			q_m=marker_transform.getRotation();
			p_m=marker_transform.getOrigin();

		}catch(tf::TransformException ex)
		{
			cout<<"An exception has occured!==="<<ex.what()<<endl;
		}
	}
	
	ros::spinOnce();
	cout<<"Transform Gripper ---> Marker"<<endl;
	cout<<"==Translation==\n"<<p_m.getX()<<" "<<p_m.getY()<<" "<<p_m.getZ()<<"\n============="<<endl;
	cout<<"==Rotation==\n"<<q_m.getX()<<" "<<q_m.getY()<<" "<<q_m.getZ()<<" "<<q_m.getW()<<"\n================"<<endl;


	if(!listener.waitForTransform("/base","/right_gripper",ros::Time(0),ros::Duration(5.0),ros::Duration(0.01),&error_msg))
	{
		cout<<"Transform not ready! "<<error_msg<<endl;
	}else
	{
		try
		{
			listener.lookupTransform("/base","/right_gripper",ros::Time(0),gripper_transform);
			q_g=gripper_transform.getRotation();
			p_g=gripper_transform.getOrigin();

		}catch(tf::TransformException ex)
		{
			cout<<"An exception has occured!==="<<ex.what()<<endl;
		}
	}
	
	ros::spinOnce();

	cout<<"Base ---> Right gripper transform"<<endl;
	cout<<"==Translation==\n"<<p_g.getX()<<" "<<p_g.getY()<<" "<<p_g.getZ()<<"\n============="<<endl;
	cout<<"==Rotation==\n"<<q_g.getX()<<" "<<q_g.getY()<<" "<<q_g.getZ()<<" "<<q_g.getW()<<"\n================"<<endl;


	/*if(!listener.waitForTransform("/base","/ar_marker_10",ros::Time(0),ros::Duration(5.0),ros::Duration(0.01),&error_msg))
	{
		cout<<"Transform not ready! "<<error_msg<<endl;
	}else
	{
		try
		{
			listener.lookupTransform("/base","/ar_marker_10",ros::Time(0),transform);
			q=transform.getRotation();
			p=transform.getOrigin();

		}catch(tf::TransformException ex)
		{
			cout<<"An exception has occured!==="<<ex.what()<<endl;
		}
	}

	cout<<"Base ---> Marker"<<endl;
	cout<<"==Translation==\n"<<p.getX()<<" "<<p.getY()<<" "<<p.getZ()<<"\n============="<<endl;
	cout<<"==Rotation==\n"<<q.getX()<<" "<<q.getY()<<" "<<q.getZ()<<" "<<q.getW()<<"\n================"<<endl;

	
	double val=p_m.getX()-p_g.getX();*/

	double val_x=p_m.getX()-p_g.getX();
	double val_y=p_m.getY()-p_g.getY();


	geometry_msgs::Pose pose;
	pose.position.x=p_g.getX()+p_m.getX();
	pose.position.y=p_g.getY();
	//pose.position.y=p_g.getY()+p_m.getY();
	pose.position.z=p_g.getZ();

	pose.orientation.x=q_g.getX();
	pose.orientation.y=q_g.getY();
	pose.orientation.z=q_g.getZ();
	pose.orientation.w=q_g.getW();

	group_right.setGoalPositionTolerance(1e-3);
	group_right.setGoalOrientationTolerance(1e-2);
	group_right.setStartState(*group_right.getCurrentState());
	group_right.setPoseTarget(pose);

	group_right.setGoalTolerance(0.03);

	cout<<"Trying to find a plan!"<<endl;


	if(group_right.plan(plan_right))
	{
		cout<<"Found a plan moving the arm!"<<endl;
		sleep(3.0);
		group_right.move();
	}else
	{
		cout<<"Plan has not been found!"<<endl;
	}



	cout<<"Job finished!"<<endl;
	sleep(2.0);
	return 0;
}

	/*



	double val=p.getZ()-0.2;

	geometry_msgs::Pose pose;
	pose.position.x=p.getX();
	pose.position.y=p.getY();
	pose.position.z=val;

	pose.orientation.x=q.getX();
	pose.orientation.y=q.getY();
	pose.orientation.z=q.getZ();
	pose.orientation.w=q.getW();

	group_right.setStartState(*group_right.getCurrentState());
	group_right.setPoseTarget(pose);

	cout<<"Looking for a plan!"<<endl;
	cout<<"Here!"<<endl;

	group_right.setGoalTolerance(0.03);

	if(group_right.plan(plan_right))
	{
		cout<<"Plan found!"<<endl;
		sleep(2.0);
		group_right.move();
	}else
	{
		cout<<"Plan has not been found!"<<endl;
	}

	cout<<"Position reached!"<<endl;
	sleep(2.0);
	ros::shutdown();
	return 0;


}

	
	/*geometry_msgs::Pose pose;
	pose.position.x=x;
	pose.position.y=y-0.05;
	pose.position.z=z;

	group_right.setStartState(*group_right.getCurrentState());

	group_right.setPositionTarget(x,y,z);

	cout<<"Trying to find a plan!"<<endl;

	if(group_right.plan(plan_right))
	{
		cout<<"Found a plan moving the arm!"<<endl;
		group_right.move();
	}else
	{
		cout<<"Plan has not been found!"<<endl;
	}



	ros::waitForShutdown();
	return 0;

}

	
	/*if(!listener.waitForTransform("/right_hand_camera","/ar_marker_9",ros::Time(0),ros::Duration(5.0),ros::Duration(0.01),&error_msg))
	{
		cout<<"Transform not ready! "<<error_msg<<endl;
	}else
	{
		try
		{
			listener.lookupTransform("/right_hand_camera","/ar_marker_10",ros::Time(0),transform);
			q=transform.getRotation();
			p=transform.getOrigin();

		}catch(tf::TransformException ex)
		{
			cout<<"An exception has occured!==="<<ex.what()<<endl;
		}
	}
	
	ros::spinOnce();


	spinner.start();
	cout<<"After spin!"<<endl;

	cout<<"==Translation==\n"<<p.getX()<<" "<<p.getY()<<" "<<p.getZ()<<"\n============="<<endl;
	cout<<"==Rotation==\n"<<q.getX()<<" "<<q.getY()<<" "<<q.getZ()<<" "<<q.getW()<<"\n================"<<endl;

	group_right.setStartState(*group_right.getCurrentState());

	geometry_msgs::Pose pose;
	pose.position.x=p.getX();
	pose.position.y=p.getY();
	pose.position.z=0.244;

	pose.orientation.x=q.getX();
	pose.orientation.y=q.getY();
	pose.orientation.z=q.getZ();
	pose.orientation.w=q.getW();



	group_right.setPoseTarget(pose);

	if(group_right.plan(plan_right))
	{
		group_right.move();
	}else
	{
		cout<<"Plan has not been found!"<<endl;
	}

	ros::waitForShutdown();
	return 0;


}

/*
int main(int argc,char**argv)
{
	ros::init(argc,argv,NODE_NAME);
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(1);	

	cout<<NODE_DESCRIPTION<<endl;

	moveit::planning_interface::MoveGroup group_right(RIGHT_ARM_PLANNING_GROUP);

	moveit::planning_interface::MoveGroup::Plan plan_right;

	spinner.start();

	group_right.setStartState(*group_right.getCurrentState());

	//moveit::planning_interface::MoveGroup group_left(LEFT_ARM_PLANNING_GROUP);
	//moveit::planning_interface::MoveGroup group_right(RIGHT_ARM_PLANNING_GROUP);

	//moveit::planning_interface::MoveGroup::Plan plan_right;
	//spinner.start();
	
	
	group_right.setPositionTarget(KNOWN_POSITION_X,KNOWN_POSITION_Y,0.28);
	group_right.setOrientationTarget(-0.14343,0.98935,-0.0016489,0.024906);
	if(group_right.plan(plan_right))
	{
        	group_right.move();
		cout<<"First position for right target reached!"<<endl;
	}else
	{
		cout<<"NO plan found for the position target of the right arm!"<<endl;
	}

	sleep(2.0);
	ros::shutdown();
/*
	group_left.setStartState(*group_left.getCurrentState());
	group_right.setStartState(*group_right.getCurrentState());
	
	
	geometry_msgs::Pose left_pose;
	left_pose.position.x=LEFT_TARGET_X;
	left_pose.position.y=LEFT_TARGET_Y;
	left_pose.position.z=LEFT_TARGET_Z;
	left_pose.orientation.x=LEFT_ORIENTATION_X;
	left_pose.orientation.y=LEFT_ORIENTATION_Y;
	left_pose.orientation.z=LEFT_ORIENTATION_Z;
	left_pose.orientation.w=LEFT_ORIENTATION_W;

	group_left.setPoseTarget(left_pose);
	/*group_left.setPositionTarget(LEFT_TARGET_X,LEFT_TARGET_Y,LEFT_TARGET_Z);
	group_right.setPositionTarget(RIGHT_TARGET_X,RIGHT_TARGET_Y,RIGHT_TARGET_Z);*/
	
	
	/*if(group_left.plan(plan_left))
	{
        	group_left.move();
		cout<<"First position for left target reached!"<<endl;
	}else
	{
		cout<<"NO plan found for the position target of the left arm!"<<endl;
	}

	sleep(2.0);	

	geometry_msgs::Pose left_pose2;
	left_pose2.position.x=LEFT_TARGET_X2;
	left_pose2.position.y=LEFT_TARGET_Y2;
	left_pose2.position.z=LEFT_TARGET_Z2;
	left_pose2.orientation.x=LEFT_ORIENTATION_X2;
	left_pose2.orientation.y=LEFT_ORIENTATION_Y2;
	left_pose2.orientation.z=LEFT_ORIENTATION_Z2;
	left_pose2.orientation.w=LEFT_ORIENTATION_W2;
	
	group_left.setPoseTarget(left_pose2);

	if(group_left.plan(plan_left))
	{
        	group_left.move();
		cout<<"Second position for left target reached!"<<endl;
	}else
	{
		cout<<"NO plan found for the position target of the left arm!"<<endl;
	}	

		
	geometry_msgs::Pose right_pose;
	right_pose.position.x=RIGHT_TARGET_X;
	right_pose.position.y=RIGHT_TARGET_Y;
	right_pose.position.z=RIGHT_TARGET_Z;
	right_pose.orientation.x=RIGHT_ORIENTATION_X;
	right_pose.orientation.y=RIGHT_ORIENTATION_Y;
	right_pose.orientation.z=RIGHT_ORIENTATION_Z;
	right_pose.orientation.w=RIGHT_ORIENTATION_W;
	
	group_right.setPoseTarget(right_pose);	

	if(group_right.plan(plan_right))
	{
		group_right.move();
		cout<<"Position for right arm reached!"<<endl;
	}else
	{
		cout<<"NO plan found for the position target of the right arm!"<<endl;
	}


	//group_left.setOrientationTarget(LEFT_ORIENTATION_X,LEFT_ORIENTATION_Y,LEFT_ORIENTATION_Z,LEFT_ORIENTATION_W);
	
	/*group.setRandomTarget();
	
	group.move();
	cout<<"Position reached!"<<endl;
	sleep(1.0);*/