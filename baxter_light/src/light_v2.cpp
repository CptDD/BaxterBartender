#include <iostream>
#include <ros/ros.h>

#include <baxter_gripper_srv/gripper_srv.h>
#include <baxter_current_pose/position.h>
#include <baxter_detect_bulb/bulb_detect.h>

#include <moveit/move_group_interface/move_group.h>

#define NODE_NAME "light_node"

using namespace std;


#define MARKER_SERVICE_NAME "position_service"
#define GRIPPER_SERVICE_NAME "gripper_service"
#define DETECT_SERVICE_NAME "detect_service"

/*******************************************************
 * Left hand home state                                *
 * 	X :0.579772 Y:0.191197 Z :0.0993143                * 
 *	Orientation --->                                   *
 *	X :0.136569 Y :0.990184 Z :0.0176215 W :0.0239423  *
 * =================================================   *
 * Right hand home state                               *
 * X :0.580194 Y:-0.192612 Z :0.0980529                *
 *	Orientation --->                                   *
 *	X :-0.135652 Y :0.99039 Z :-0.0158881 W :0.0217533 *
 *******************************************************/




geometry_msgs::Pose go_up_left(int when=0)
{
	geometry_msgs::Pose pose;


	pose.position.x=0.470336;
	pose.position.y=0.621562;
	pose.position.z=0.312792;

	pose.orientation.x=0.301255;
	pose.orientation.y=-0.953306;
	pose.orientation.z=-0.00703267;
	pose.orientation.w=0.0200823;

	return pose;
}


geometry_msgs::Pose go_tester()
{

	geometry_msgs::Pose pose;
	pose.position.x=0.732617;
	pose.position.y=0.134935;
	pose.position.z=0.111202;

	pose.orientation.x=0.231323;
	pose.orientation.y=0.972512;
	pose.orientation.z=0.0197414;
	pose.orientation.w=0.017899;

	return pose;

}



geometry_msgs::Pose go_up_left_2(int when=0)
{

	geometry_msgs::Pose pose;



	pose.position.x=0.55012;
	pose.position.y=0.77427;
	pose.position.z=0.337524;


	pose.orientation.x=-0.266432;
	pose.orientation.y=0.696694;
	pose.orientation.z=0.261117;
	pose.orientation.w=0.612739;

	if(when!=0)
	{
		pose.position.y-=0.3;
	}

	return pose;
}


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
	

	pose.position.x=0.621739;
	pose.position.y=0.704905;
	pose.position.z=0.293084;

	pose.orientation.x=0.287159;
	pose.orientation.y=-0.95764;
	pose.orientation.z=0.0176938;
	pose.orientation.w=0.012359;

		/*pose.position.x=0.521501;
		pose.position.y=0.771216; 
		pose.position.z=0.294003;

		pose.orientation.x=0.389772;
		pose.orientation.y=-0.920555;
		pose.orientation.z=0.0235939;
		pose.orientation.w=0.00997334;*/
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

geometry_msgs::Pose go_pre_pick_2()
{
	geometry_msgs::Pose pose;
	pose.position.x=0.586557;
	pose.position.y=0.843761;
	pose.position.z=0.340047;

	pose.orientation.x=-0.232637;
	pose.orientation.y=0.704651;
	pose.orientation.z=0.255763;
	pose.orientation.w=0.619622;

	return pose;


}

void go_pick(moveit::planning_interface::MoveGroup &group,bool way=0)
{
	geometry_msgs::Pose current_pose=group.getCurrentPose().pose;
	if(!way)
	{
		current_pose.position.z-=0.03;
	}else
	{
		current_pose.position.z+=0.05;
	}
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

void execute_move(moveit::planning_interface::MoveGroup &group,bool special=false)
{

	//group.setGoalPositionTolerance(1e-2);
	if(!special)
	{
		group.setGoalPositionTolerance(0.005);
		group.setMaxVelocityScalingFactor(0.5);

	}else
	{
		group.setGoalPositionTolerance(0.001);
		group.setMaxVelocityScalingFactor(0.25);

	}
	group.setGoalOrientationTolerance(1e-3);
	group.setStartState(*group.getCurrentState());
	
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

geometry_msgs::Pose first_storage_reconstruct(ros::NodeHandle &nh)
{
	double p_x,p_y,p_z,o_x,o_y,o_z,o_w;

	geometry_msgs::Pose pose;

	nh.getParam("/first_p_x",p_x);
	nh.getParam("/first_p_y",p_y);
	nh.getParam("/first_p_z",p_z);

	nh.getParam("/first_o_x",o_x);
	nh.getParam("/first_o_x",o_y);
	nh.getParam("/first_o_x",o_z);
	nh.getParam("/first_o_x",o_w);

	pose.position.x=p_x;
	pose.position.y=p_y;
	pose.position.z=p_z;

	pose.orientation.x=o_x;
	pose.orientation.y=o_y;
	pose.orientation.z=o_z;
	pose.orientation.w=o_w;

	ROS_INFO("First storage reconstructed!");

	return pose;
}


geometry_msgs::Pose second_storage_reconstruct(ros::NodeHandle &nh)
{
	double p_x,p_y,p_z,o_x,o_y,o_z,o_w;

	geometry_msgs::Pose pose;

	nh.getParam("/second_p_x",p_x);
	nh.getParam("/second_p_y",p_y);
	nh.getParam("/second_p_z",p_z);

	nh.getParam("/second_o_x",o_x);
	nh.getParam("/second_o_x",o_y);
	nh.getParam("/second_o_x",o_z);
	nh.getParam("/second_o_x",o_w);

	pose.position.x=p_x;
	pose.position.y=p_y;
	pose.position.z=p_z;

	pose.orientation.x=o_x;
	pose.orientation.y=o_y;
	pose.orientation.z=o_z;
	pose.orientation.w=o_w;

	ROS_INFO("Second storage reconstructed!");


	return pose;
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
	ros::ServiceClient gripper_client=nh.serviceClient<baxter_gripper_srv::gripper_srv>(GRIPPER_SERVICE_NAME);

	spinner.start();

	ROS_INFO("===Position reconstruction phase===");

	first_storage=first_storage_reconstruct(nh);
	second_storage=second_storage_reconstruct(nh);


	group_right.setPoseTarget(go_home("right"));
	execute_move(group_right);

	group_left.setPoseTarget(go_home("left"));
	execute_move(group_left);

	
	ROS_INFO("===Light bulb pick phase===");

	group_left.setPoseTarget(go_up_left());
	execute_move(group_left);

	group_left.setPoseTarget(go_pre_pick("left"));
	execute_move(group_left,true);
	sleep(2.0);

	go_pick(group_left);
	execute_move(group_left,true);

	sleep(2.0);

	grip_srv.request.gripper.data="left";
	grip_srv.request.action.data=0;

	if(gripper_client.call(grip_srv))
	{
		cout<<"Left gripper closed!"<<endl;
	}else
	{
		cout<<"Left gripper did not close!"<<endl;
	}

	go_pick(group_left,true);
	execute_move(group_left);

	group_left.setPoseTarget(go_up_left());
	execute_move(group_left);

	group_left.setPoseTarget(go_home("left"));
	execute_move(group_left);

	group_left.setPoseTarget(go_tester());
	execute_move(group_left);

	sleep(2.0);

	current_pose_left=get_current_pose(group_left);
	current_pose_left.position.z-=0.04;

	group_left.setPoseTarget(current_pose_left);
	execute_move(group_left);

	grip_srv.request.gripper.data="left";
	grip_srv.request.action.data=1;

	if(gripper_client.call(grip_srv))
	{
		cout<<"Left gripper opened!"<<endl;
	}else
	{
		cout<<"Left gripper did not open!"<<endl;
	}

	group_left.setPoseTarget(go_home("left"));
	execute_move(group_left);

	group_left.setPoseTarget(go_up_left());
	execute_move(group_left);

	group_right.setPoseTarget(go_tester());
	execute_move(group_right);

	current_pose_right=get_current_pose(group_right);
	current_pose_right.position.z-=0.06;

	group_right.setPoseTarget(current_pose_right);
	execute_move(group_right);

	grip_srv.request.gripper.data="right";
	grip_srv.request.action.data=0;

	if(gripper_client.call(grip_srv))
	{
		cout<<"Right gripper closed!"<<endl;
	}else
	{
		cout<<"Right gripper did not close!"<<endl;
	}

	current_pose_right=get_current_pose(group_right);
	current_pose_right.position.z+=0.1;

	group_right.setPoseTarget(current_pose_right);
	execute_move(group_right);


	int what;

	cout<<"Big=0 or small=1?";
	cin>>what;

	current_pose_right=get_current_pose(group_right);

	if(what==0)
	{
		current_pose_right.position.x=first_storage.position.x;
		current_pose_right.position.y=first_storage.position.y;
	}else
	{
		current_pose_right.position.x=second_storage.position.x;
		current_pose_right.position.y=second_storage.position.y;
	}

	group_right.setPoseTarget(current_pose_right);
	execute_move(group_right);



	grip_srv.request.gripper.data="right";
	grip_srv.request.action.data=1;

	if(gripper_client.call(grip_srv))
	{
		cout<<"Right gripper opened"<<endl;
	}else
	{
		cout<<"Right gripper did not open!"<<endl;
	}




	/*current_pose_left=get_current_pose(group_left);
	current_pose_left.position.y-=0.1;*/

	
	//group_left.setPoseTarget(go_home("left"));
	//execute_move(group_left);

	//group_left.setPoseTarget(current_pose_left);
	//execute_move(group_left);

	//group_left.setPoseTarget(go_tester());
	//execute_move(group_left);

	//sleep(2.0);

	/*current_pose_left=get_current_pose(group_left);
	current_pose_left.position.z-=0.03;
	group_left.setPoseTarget(current_pose_left);
	execute_move(group_left,true);

	sleep(2.0);

	grip_srv.request.gripper.data="left";
	grip_srv.request.action.data=1;

	if(gripper_client.call(grip_srv))
	{
		cout<<"Left gripper opened!"<<endl;
	}else
	{
		cout<<"Left gripper did not open!"<<endl;
	}

	current_pose_left=get_current_pose(group_left);
	current_pose_left.position.z+=0.05;
	group_left.setPoseTarget(current_pose_left);
	execute_move(group_left);

	group_left.setPoseTarget(go_home("left"));
	execute_move(group_left);

	group_left.setPoseTarget(go_up_left());
	execute_move(group_left);

	/*group_right.setPoseTarget(go_home("right"));
	execute_move(group_right);

	current_pose_right=get_current_pose(group_right);

	current_pose_right.position.x=hole_target.position.x;
	current_pose_right.position.y=hole_target.position.y;


	group_right.setPoseTarget(current_pose_right);
	execute_move(group_right);

	
	current_pose_right=get_current_pose(group_right);
	current_pose_right.position.z=hole_target.position.z+0.2;

	group_right.setPoseTarget(current_pose_right);
	execute_move(group_right);
	/*

	grip_srv.request.gripper.data="right";
	grip_srv.request.action.data=0;

	if(gripper_client.call(grip_srv))
	{
		cout<<"Right gripper closed!"<<endl;
	}else
	{
		cout<<"Right gripper did not close!"<<endl;
	}

	current_pose_right=get_current_pose(group_right);
	current_pose_right.position.z+=0.15;

	group_right.setPoseTarget(current_pose_right);
	execute_move(group_right);

	group_right.setPoseTarget(go_home("right"));
	execute_move(group_right);

	current_pose_right=get_current_pose(group_right);

	current_pose_right.position.x=storage_box.position.x;
	current_pose_right.position.y=storage_box.position.y;

	group_right.setPoseTarget(current_pose_right);
	execute_move(group_right);

	current_pose_right=get_current_pose(group_right);
	current_pose_right.position.z=storage_box.position.z+0.08;
	group_right.setPoseTarget(current_pose_right);
	execute_move(group_right);


	grip_srv.request.gripper.data="right";
	grip_srv.request.action.data=1;

	if(gripper_client.call(grip_srv))
	{
		cout<<"Right gripper opened!"<<endl;
	}else
	{
		cout<<"Right gripper did not open!"<<endl;
	}

	group_right.setPoseTarget(go_home("right"));
	execute_move(group_right);*/



	sleep(2.0);
	cout<<"Job finished!"<<endl;

	ros::shutdown();


	return 0;
}