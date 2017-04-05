#include <iostream>
#include <ros/ros.h>
#include <baxter_gripper_srv/gripper_srv.h>
#include <baxter_current_pose/position.h>
#include <moveit/move_group_interface/move_group.h>

#include <baxter_core_msgs/CloseCamera.h>
#include <baxter_core_msgs/ListCameras.h>
#include <baxter_core_msgs/OpenCamera.h>

#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>

#define NODE_NAME "learning_node"
#define MARKER_SERVICE_NAME "position_service"
#define GRIPPER_SERVICE_NAME "gripper_service"
#define CAMERA_CLOSE_SERVICE_NAME "/cameras/close"
#define CAMERA_OPEN_SERVICE_NAME "/cameras/open"


using namespace std;




geometry_msgs::Pose go_first_storage(ros::ServiceClient client,int observe=0)
{
	geometry_msgs::Pose pose;

	baxter_current_pose::position srv;
	observe==0?srv.request.arm.data="/hole_target":srv.request.arm.data="/ar_marker_3";

	cout<<"Requesting first target position . . ."<<endl;
	if(client.call(srv))
	{
		cout<<"First storage target acquired!"<<endl;

		pose=srv.response.current_pose;

		cout<<"The pose of the storage is :"<<endl;
		cout<<"X :"<<pose.position.x<<" Y :"<<pose.position.y<<" Z :"<<pose.position.z<<endl;
		cout<<"X :"<<pose.orientation.x<<" Y :"<<pose.orientation.y<<" Z :"<<pose.orientation.z<<" W :"<<pose.orientation.w<<endl;
	}

	return pose;

}

geometry_msgs::Pose go_second_storage(ros::ServiceClient client,int observe=0)
{
	geometry_msgs::Pose pose;

	baxter_current_pose::position srv;
	observe==0?srv.request.arm.data="/hole_target":srv.request.arm.data="/ar_marker_0";

	cout<<"Requesting second target position . . ."<<endl;
	if(client.call(srv))
	{
		cout<<"Second storage target acquired!"<<endl;

		pose=srv.response.current_pose;

		cout<<"The pose of the storage is :"<<endl;
		cout<<"X :"<<pose.position.x<<" Y :"<<pose.position.y<<" Z :"<<pose.position.z<<endl;
		cout<<"X :"<<pose.orientation.x<<" Y :"<<pose.orientation.y<<" Z :"<<pose.orientation.z<<" W :"<<pose.orientation.w<<endl;
	}

	return pose;
}



geometry_msgs::Pose go_observe()
{
	geometry_msgs::Pose pose;

	pose.position.x=0.660664;
	pose.position.y=-0.493151;
	pose.position.z=0.377511;

    pose.orientation.x=-0.138625;
    pose.orientation.y=-0.990107;
    pose.orientation.z=-0.0201211;
    pose.orientation.w=0.00815503;

    return pose;

}

geometry_msgs::Pose go_home(string arm,bool observe=false)
{
	geometry_msgs::Pose pose;

	if(arm=="left")
	{



		if(!observe)
		{
	
			pose.position.x=0.573391;   
			pose.position.y=0.29437;
			pose.position.z=0.092628;

			pose.orientation.x=-0.0767556;
			pose.orientation.y=-0.997035;
			pose.orientation.z=-0.00553669;
			pose.orientation.w=0.000247086;
		}else
		{

			pose.position.x=0.670432;
			pose.position.y=0.208607;
			pose.position.z=0.298132;
	
			pose.orientation.x=0.908886;
			pose.orientation.y=-0.416915;
			pose.orientation.z=0.00208194;
			pose.orientation.w=0.0102218;

			/*pose.position.x=0.643083;
			pose.position.y=0.273531;
			pose.position.z=0.226807;
			
			pose.orientation.x=0.973938;
			pose.orientation.y=-0.226406;
			pose.orientation.z=0.0117925;
			pose.orientation.w=0.00684724;*/
		}

	}else
	{

		pose.position.x=0.571731;
		pose.position.y=-0.184004;
		pose.position.z=0.269815;

		/*pose.orientation.x=-0.136913;
		pose.orientation.y=0.990232;
		pose.orientation.z=-0.0092733;
		pose.orientation.w=0.0246997;*/


		if(!observe)
		{

			pose.position.x=0.704264;
			pose.position.y=-0.203243;
			pose.position.z=0.278785;

            pose.orientation.x=0.10500;
            pose.orientation.y=-0.99435;
            pose.orientation.z=-0.0101156;
            pose.orientation.w=0.0118141;


        



		}else
		{

			pose.position.x=0.647823;
			pose.position.y=-0.108554;
			pose.position.z=0.35;


			pose.orientation.x=-0.97416;
			pose.orientation.y=-0.222648;
			pose.orientation.z=0.0275387;
			pose.orientation.w=0.0261027;
		}



		/*pose.position.x=0.580194;
		pose.position.y=-0.192612;
		pose.position.z=0.0980529;

		pose.orientation.x=-0.135652;
		pose.orientation.y=0.99039;
		pose.orientation.z=-0.0158881;
		pose.orientation.w=0.0217533;*/
	}

	return pose;
}


void go_neutral_right(moveit::planning_interface::MoveGroup &group)
{
	group.setStartState(*group.getCurrentState());
	geometry_msgs::Pose current_pose=group.getCurrentPose().pose;

	current_pose.position.x=0.33563;
	current_pose.position.y=-0.87137;
	group.setPoseTarget(current_pose);
}

void go_neutral_left(moveit::planning_interface::MoveGroup &group)
{
	group.setStartState(*group.getCurrentState());
	geometry_msgs::Pose current_pose=group.getCurrentPose().pose;

	current_pose.position.x=0.510488;
	current_pose.position.y=0.380421;
	current_pose.position.z=-0.0697811;
	group.setPoseTarget(current_pose);
}

void go_up(moveit::planning_interface::MoveGroup &group)
{
	geometry_msgs::Pose current_pose=group.getCurrentPose().pose;
	current_pose.position.z+=0.1;
	group.setPoseTarget(current_pose);
}

geometry_msgs::Pose get_current_pose(moveit::planning_interface::MoveGroup &group)
{
	group.setStartState(*group.getCurrentState());
	return group.getCurrentPose().pose;
}

void execute_move(moveit::planning_interface::MoveGroup &group)
{

	//group.setGoalPositionTolerance(1e-2);
	group.setGoalPositionTolerance(0.005);
	group.setGoalOrientationTolerance(1e-3);
	group.setStartState(*group.getCurrentState());
	group.setMaxVelocityScalingFactor(0.5);
	//group.setMaxAccelerationScalingFactor(0.5);

	moveit::planning_interface::MoveGroup::Plan plan;

	if(group.plan(plan))
	{
		cout<<"Found a plan moving the arm!"<<endl;
		sleep(1.0);
		group.move();
	}else
	{
		cout<<"Plan has not been found!"<<endl;
	}
	
}


int main(int argc,char**argv)
{
	ros::init(argc,argv,NODE_NAME);
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(5);
	spinner.start();

	moveit::planning_interface::MoveGroup group_right("right_arm");
	moveit::planning_interface::MoveGroup group_left("left_arm");

	moveit::planning_interface::MoveGroup::Plan plan_right,plan_left;
	geometry_msgs::Pose current_pose_right,current_pose_left,first_storage,second_storage;

	baxter_gripper_srv::gripper_srv grip_srv;

	ros::ServiceClient marker_client=nh.serviceClient<baxter_current_pose::position>(MARKER_SERVICE_NAME);

	spinner.start();

	group_right.setPoseTarget(go_home("right"));
	execute_move(group_right);

	group_left.setPoseTarget(go_home("left"));
	execute_move(group_left);

	ROS_INFO("===First storage container learning phase===");

	group_right.setPoseTarget(go_observe());
	execute_move(group_right);


	/*ROS_INFO("Moving right arm to neutral position");
	go_neutral_right(group_right);
	execute_move(group_right);*/

	first_storage=go_first_storage(marker_client,1);

	current_pose_right=get_current_pose(group_right);

	current_pose_right.position.x=first_storage.position.x;
	current_pose_right.position.y=first_storage.position.y;

	group_right.setPoseTarget(current_pose_right);
	execute_move(group_right);

	sleep(3.0);

	first_storage=go_first_storage(marker_client,1);

	nh.setParam("/first_p_x",first_storage.position.x);
	nh.setParam("/first_p_y",first_storage.position.y);
	nh.setParam("/first_p_z",first_storage.position.z);

	nh.setParam("/first_o_x",first_storage.orientation.x);
	nh.setParam("/first_o_y",first_storage.orientation.y);
	nh.setParam("/first_o_z",first_storage.orientation.z);
	nh.setParam("/first_o_w",first_storage.orientation.w);

	ROS_INFO("First storage parameters loaded!");

	group_right.setPoseTarget(go_observe());
	execute_move(group_right);

	ROS_INFO("===Second storage container learning phase===");

	second_storage=go_second_storage(marker_client,1);

	current_pose_right=get_current_pose(group_right);

	current_pose_right.position.x=second_storage.position.x;
	current_pose_right.position.y=second_storage.position.y;

	group_right.setPoseTarget(current_pose_right);
	execute_move(group_right);

	second_storage=go_second_storage(marker_client,1);

	nh.setParam("/second_p_x",second_storage.position.x);
	nh.setParam("/second_p_y",second_storage.position.y);
	nh.setParam("/second_p_z",second_storage.position.z);

	nh.setParam("/second_o_x",second_storage.orientation.x);
	nh.setParam("/second_o_y",second_storage.orientation.y);
	nh.setParam("/second_o_z",second_storage.orientation.z);
	nh.setParam("/second_o_w",second_storage.orientation.w);

	ROS_INFO("Second storage parameters loaded!");

	group_right.setPoseTarget(go_home("right"));
	execute_move(group_right);




	sleep(2.0);
	cout<<"Job finished!"<<endl;

	ros::shutdown();


	return 0;
}

