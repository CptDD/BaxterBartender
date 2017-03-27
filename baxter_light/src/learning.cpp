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

using namespace std;

#define MARKER_SERVICE_NAME "position_service"
#define GRIPPER_SERVICE_NAME "gripper_service"
#define CAMERA_CLOSE_SERVICE_NAME "/cameras/close"
#define CAMERA_OPEN_SERVICE_NAME "/cameras/open"




geometry_msgs::Pose go_light_bulb_tester(ros::ServiceClient client,int observe=0)
{
	geometry_msgs::Pose pose;

	baxter_current_pose::position srv;
	observe==0?srv.request.arm.data="/hole_target":srv.request.arm.data="/ar_marker_9";

	cout<<"Requesting light bulb position ... "<<endl;
	if(client.call(srv))
	{
		cout<<"Light bulb position aquired!"<<endl;
		pose=srv.response.current_pose;

		cout<<"The pose of the marker is :"<<endl;
		cout<<"X :"<<pose.position.x<<" Y :"<<pose.position.y<<" Z :"<<pose.position.z<<endl;
		cout<<"X :"<<pose.orientation.x<<" Y :"<<pose.orientation.y<<" Z :"<<pose.orientation.z<<" W :"<<pose.orientation.w<<endl;
	}

	return pose;
}

geometry_msgs::Pose go_temporary_box(ros::ServiceClient client,int observe=0)
{
	geometry_msgs::Pose pose;

	baxter_current_pose::position srv;
	observe==0?srv.request.arm.data="/temp_target":srv.request.arm.data="/ar_marker_8";

	cout<<"Requesting temporary box position ... "<<endl;
	if(client.call(srv))
	{
		cout<<"Temporary box  position aquired!"<<endl;
		pose=srv.response.current_pose;

		cout<<"The pose of the marker is :"<<endl;
		cout<<"X :"<<pose.position.x<<" Y :"<<pose.position.y<<" Z :"<<pose.position.z<<endl;
		cout<<"X :"<<pose.orientation.x<<" Y :"<<pose.orientation.y<<" Z :"<<pose.orientation.z<<" W :"<<pose.orientation.w<<endl;
	}

	return pose;
}

geometry_msgs::Pose go_storage_box(ros::ServiceClient client,int observe=0)
{
	geometry_msgs::Pose pose;

	baxter_current_pose::position srv;
	observe==0?srv.request.arm.data="/storage_target":srv.request.arm.data="/ar_marker_10";


	cout<<"Requesting storage box position ... "<<endl;
	if(client.call(srv))
	{
		cout<<"Storage box position acquired!"<<endl;
		pose=srv.response.current_pose;

		cout<<"The pose of the marker is :"<<endl;
		cout<<"X :"<<pose.position.x<<" Y :"<<pose.position.y<<" Z :"<<pose.position.z<<endl;
		cout<<"X :"<<pose.orientation.x<<" Y :"<<pose.orientation.y<<" Z :"<<pose.orientation.z<<" W :"<<pose.orientation.w<<endl;
	}

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

geometry_msgs::Pose go_pre_pick(string arm)
{

	geometry_msgs::Pose pose;

	if(arm=="left")
	{
	
		pose.position.x=0.521501;
		pose.position.y=0.771216; 
		pose.position.z=0.294003;

		pose.orientation.x=0.389772;
		pose.orientation.y=-0.920555;
		pose.orientation.z=0.0235939;
		pose.orientation.w=0.00997334;
	}else
	{
		pose.position.x=0.580194;
		pose.position.y=-0.192612;
		pose.position.z=0.0980529;

		pose.orientation.x=-0.135652;
		pose.orientation.y=0.99039;
		pose.orientation.z=-0.0158881;
		pose.orientation.w=0.0217533;
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
	
	cout<<"Now going home"<<endl;
}

void rotate_right(moveit::planning_interface::MoveGroup &group,int front=0)
{
	group.setStartState(*group.getCurrentState());
	geometry_msgs::Pose pose=group.getCurrentPose().pose;

	if(!front)
	{
		 pose.orientation.x=0.10500;
         pose.orientation.y=-0.99435;
         pose.orientation.z=-0.0101156;
         pose.orientation.w=0.0118141;
	}else
	{
		pose.orientation.x=-0.97416;
		pose.orientation.y=-0.222648;
		pose.orientation.z=0.0275387;
		pose.orientation.w=0.0261027;
	}

	group.setPoseTarget(pose);


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
	geometry_msgs::Pose current_pose_right,current_pose_left,hole_target,storage_box,temporary_box;

	baxter_gripper_srv::gripper_srv grip_srv;

	ros::ServiceClient marker_client=nh.serviceClient<baxter_current_pose::position>(MARKER_SERVICE_NAME);

	spinner.start();

	/*group_right.setPoseTarget(go_home("right"));
	execute_move(group_right);*/
	group_left.setPoseTarget(go_home("left",true));
	execute_move(group_left);

	ROS_INFO("===Light bulb tester learning phase===");

	/*ROS_INFO("Moving right arm to neutral position");
	go_neutral_right(group_right);
	execute_move(group_right);*/

	hole_target=go_light_bulb_tester(marker_client);

	current_pose_left=get_current_pose(group_left);

	current_pose_left.position.x=hole_target.position.x;
	current_pose_left.position.y=hole_target.position.y;


	group_left.setPoseTarget(current_pose_left);
	execute_move(group_left);

	sleep(3.0);
	hole_target=go_light_bulb_tester(marker_client);

	nh.setParam("/hole_p_x",hole_target.position.x);
	nh.setParam("/hole_p_y",hole_target.position.y);
	nh.setParam("/hole_p_z",hole_target.position.z);

	nh.setParam("/hole_o_x",hole_target.orientation.x);
	nh.setParam("/hole_o_y",hole_target.orientation.y);
	nh.setParam("/hole_o_z",hole_target.orientation.z);
	nh.setParam("/hole_o_w",hole_target.orientation.w);

	ROS_INFO("Light bulb tester parameters loaded");

	ROS_INFO("===Storage box learning phase===");

	ROS_INFO("Moving the left arm to neutral");

	go_neutral_left(group_left);
	execute_move(group_left);

	/*group_right.setPoseTarget(go_home("right"));
	execute_move(group_right);

	storage_box=go_storage_box(marker_client);

	current_pose_right=get_current_pose(group_right);

	current_pose_right.position.x=storage_box.position.x;
	current_pose_right.position.y=storage_box.position.y;


	group_right.setPoseTarget(current_pose_right);
	execute_move(group_right);

	storage_box=go_storage_box(marker_client);

	nh.setParam("/storage_p_x",storage_box.position.x);
	nh.setParam("/storage_p_y",storage_box.position.y);
	nh.setParam("/storage_p_z",storage_box.position.z);

	nh.setParam("/storage_o_x",storage_box.orientation.x);
	nh.setParam("/storage_o_y",storage_box.orientation.y);
	nh.setParam("/storage_o_z",storage_box.orientation.z);
	nh.setParam("/storage_o_w",storage_box.orientation.w);*/





	sleep(2.0);
	cout<<"Job finished!"<<endl;

	ros::shutdown();


	return 0;
}

