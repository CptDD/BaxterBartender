#include <iostream>
#include <ros/ros.h>
#include <baxter_current_pose/position.h>
#include <moveit/move_group_interface/move_group.h>


using namespace std;

#define NODE_NAME "position_client"
#define SERVICE_NAME "position_service"

int main(int argc,char**argv)
{
	ros::init(argc,argv,NODE_NAME);
	ros::NodeHandle nh;

	ros::ServiceClient client=nh.serviceClient<baxter_current_pose::position>(SERVICE_NAME);

	moveit::planning_interface::MoveGroup group("right_arm");
	moveit::planning_interface::MoveGroup::Plan plan;

	ros::AsyncSpinner spinner(1);
	spinner.start();


	baxter_current_pose::position srv;

	srv.request.arm.data="left";

	geometry_msgs::Pose pose;

	if(client.call(srv))
	{
		cout<<"Success!"<<endl;

		pose=srv.response.current_pose;

		cout<<"X :"<<pose.position.x<<" Y :"<<pose.position.y<<"Z :"<<pose.position.z<<endl;
	}else
	{
		cout<<"Not success!"<<endl;
	}

	geometry_msgs::Pose current_pose=group.getCurrentPose().pose;

	current_pose.position.x=pose.position.x;
	current_pose.position.y=pose.position.y;


	group.setPoseTarget(current_pose);

	group.setGoalPositionTolerance(1e-3);
	group.setGoalOrientationTolerance(1e-2);
	group.setStartState(*group.getCurrentState());


	if(group.plan(plan))
	{
		cout<<"Found a plan moving the arm!"<<endl;
		sleep(3.0);
		group.move();
	}else
	{
		cout<<"Plan has not been found!"<<endl;
	}
	
	cout<<"Now going home"<<endl;

	return 0;
}