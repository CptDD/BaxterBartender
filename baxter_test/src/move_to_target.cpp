#include <iostream>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>



using namespace std;

#define NODE_DESCRIPTION "A node that moves the arms to specified positions"
#define NODE_NAME "manipulation_group_mover"

#define RIGHT_ARM_PLANNING_GROUP "right_arm"
#define LEFT_ARM_PLANNING_GROUP "left_arm"
#define BOTH_ARMS_PLANNING_GROUP "both_arms"

#define LEFT_TARGET_X 1.00977
#define LEFT_TARGET_Y 0.183786
#define LEFT_TARGET_Z 0.0735425

#define LEFT_ORIENTATION_X 0.114125
#define LEFT_ORIENTATION_Y 0.731938
#define LEFT_ORIENTATION_Z -0.0833392
#define LEFT_ORIENTATION_W 0.666556


#define RIGHT_TARGET_X 0.837469
#define RIGHT_TARGET_Y -0.220404
#define RIGHT_TARGET_Z 0.025409

#define RIGHT_ORIENTATION_X -0.133733
#define RIGHT_ORIENTATION_Y  0.813764
#define RIGHT_ORIENTATION_Z 0.0809839
#define RIGHT_ORIENTATION_W 0.559772


/***********
 * Beer bottle caught good values 
 * Position :0.969499, -0.241225, 0.194091
 * Orientation: -0.11632, 0.749371, 0.080213, 0.646899
 *
 **********/


int main(int argc,char**argv)
{
	ros::init(argc,argv,NODE_NAME);
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(1);

	moveit::planning_interface::MoveGroup group_left(LEFT_ARM_PLANNING_GROUP);
	moveit::planning_interface::MoveGroup group_right(RIGHT_ARM_PLANNING_GROUP);

	moveit_msgs::CollisionObject remove_object;
	remove_object.id = "beer_pint.stl";
	remove_object.header.frame_id = "base";
	remove_object.operation = remove_object.REMOVE;


	
	

	moveit::planning_interface::MoveGroup::Plan plan_left,plan_right;
	
	spinner.start();
	
	
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
	
	
	if(group_left.plan(plan_left))
	{
        	group_left.move();
		cout<<"Position for left target reached!"<<endl;
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
	
	sleep(5.0);
	

	ros::shutdown();

	return 0;
		
}
