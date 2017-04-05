#include <iostream>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>

using namespace std;



/*position --->
X :0.625291 Y:0.396352 Z :0.472943
Orientation --->
X :-0.297814 Y :0.954619 Z :-0.00267946 W :0.00156113

Position --->
X :0.603614 Y:0.642951 Z :0.277305
Orientation --->
X :0.295642 Y :-0.955097 Z :0.00689242 W :0.0183568


*/





int main(int argc,char**argv)
{
	ros::init(argc,argv,"current_pose");


	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	moveit::planning_interface::MoveGroup group_right("right_arm");
	moveit::planning_interface::MoveGroup group_left("left_arm");


	group_right.setStartState(*group_right.getCurrentState());
	group_left.setStartState(*group_left.getCurrentState());


	//group.setStartStateToCurrentState();

	geometry_msgs::Pose pose_right=group_right.getCurrentPose().pose;
	geometry_msgs::Pose pose_left=group_left.getCurrentPose().pose;

	cout<<"Position Right--->"<<endl;

	cout<<"X :"<<pose_right.position.x<<" Y:"<<pose_right.position.y<<" Z :"<<pose_right.position.z<<endl;

	cout<<"Orientation Right--->"<<endl;
	cout<<"X :"<<pose_right.orientation.x<<" Y :"<<pose_right.orientation.y<<" Z :"<<pose_right.orientation.z<<" W :"<<pose_right.orientation.w<<endl;

	cout<<"Position left--->"<<endl;

	cout<<"X :"<<pose_left.position.x<<" Y:"<<pose_left.position.y<<" Z :"<<pose_left.position.z<<endl;

	cout<<"Orientation left--->"<<endl;
	cout<<"X :"<<pose_left.orientation.x<<" Y :"<<pose_left.orientation.y<<" Z :"<<pose_left.orientation.z<<" W :"<<pose_left.orientation.w<<endl;




	sleep(2.0);
	ros::shutdown();

	return 0;
}