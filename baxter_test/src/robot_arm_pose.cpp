#include <iostream>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>


#define NODE_DESCRIPTION "A node that reads the pose of the end effector"
#define NODE_NAME "pose_reader"

#define RIGHT_ARM_PLANNING_GROUP "right_arm"
#define LEFT_ARM_PLANNING_GROUP "left_arm"
#define BOTH_ARMS_PLANNING_GROUP "both_arms"




/******************************************************
 * Beer bottle's reference pose(right arm)            *
 * Position  X -->0.908972                            *
 *           Y -->-1.10398			      *
 *           Z -->0.320976                            *
 ******************************************************
 * Orientation X -->0.270599                          *
 *             Y -->0.653281                          *
 *             Z -->-0.270599                         *
 *             W -->0.653281                          *
 *                                                    *
 ******************************************************/

/******************************************************
 * Beer pint's reference pose(left arm)               *
 * Position  X -->0.908972                            *
 *           Y -->-1.10398			      *
 *           Z -->0.320976                            *
 ******************************************************
 * Orientation X -->0.270599                          *
 *             Y -->0.653281                          *
 *             Z -->-0.270599                         *
 *             W -->0.653281                          *
 *                                                    *
 ******************************************************/

 


using namespace std;

void show_pose(const geometry_msgs::PoseStamped pose)
{
	cout<<setw(20)<<"---"<<setw(15)<<"Position"<<setw(10)<<"---"<<endl;
	cout<<setw(30)<<"X -->"<<setw(5)<<pose.pose.position.x<<endl;
	cout<<setw(30)<<"Y -->"<<setw(5)<<pose.pose.position.y<<endl;
	cout<<setw(30)<<"Z -->"<<setw(5)<<pose.pose.position.z<<endl;
	cout<<setw(20)<<"---"<<setw(15)<<"Orientation"<<setw(10)<<"---"<<endl;
        cout<<setw(30)<<"X -->"<<setw(5)<<pose.pose.orientation.x<<endl;
	cout<<setw(30)<<"Y -->"<<setw(5)<<pose.pose.orientation.y<<endl;
	cout<<setw(30)<<"Z -->"<<setw(5)<<pose.pose.orientation.z<<endl;
	cout<<setw(30)<<"W -->"<<setw(5)<<pose.pose.orientation.w<<endl;
}


int main(int argc,char **argv)
{
	ros::init(argc,argv,NODE_NAME);
	cout<<"We are here to make some noise! "<<NODE_DESCRIPTION<<endl;
	
	ros::NodeHandle nh;
        ros::AsyncSpinner spinner(1);
        spinner.start();
        
	moveit::planning_interface::MoveGroup group(LEFT_ARM_PLANNING_GROUP);
	
	
	
	group.setStartState(*group.getCurrentState());
	//group.setEndEffectorLink("left_gripper");

	cout<<"HH :"<<group.getEndEffectorLink()<<endl;

	/*tf::TransformListener listener;
	tf::StampedTransform transform;
	try{
      		 listener.lookupTransform("/block_link0", "/left_gripper",ros::Time(0), transform);
     	}
      	catch (tf::TransformException ex){
      		ROS_ERROR("%s",ex.what());
       }*/
	

		

	
	geometry_msgs::PoseStamped pose = group.getCurrentPose("left_gripper");
    	pose.header.stamp = ros::Time::now();
   	double x = pose.pose.position.x; 
    	double y = pose.pose.position.y;
    	double z = pose.pose.position.z;
	double w=pose.pose.orientation.w;
	double ox=pose.pose.orientation.x;
	double oy=pose.pose.orientation.y;
	double oz=pose.pose.orientation.z;
    	ROS_INFO("Move to : x=%f, y=%f, z=%f",x,y,z);	
	ROS_INFO("With Pose : x=%f, y=%f, z=%f, w=%f",ox,oy,oz,w);	
	
	
	
	
	
	
	ros::shutdown();
	
	return 0;
}



