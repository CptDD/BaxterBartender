#include <iostream>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <dynamic_reconfigure/server.h>
#include <baxter_test/rotationConfig.h>

using namespace std;

#define DUMMY_MARKER "/dummy_maker"
#define TARGET "/target"

double angle_rotation,displacement_x,displacement_y,target_x,target_y;


void callback(baxter_test::rotationConfig &config, uint32_t level) {
 	
	angle_rotation=config.angle_rotation;
	displacement_x=config.marker_x;
	displacement_y=config.marker_y;
	target_x=config.target_x;
	target_y=config.target_y;

	ROS_INFO("New angle rotation :%g",angle_rotation);
	ROS_INFO("New displacement x:%g",angle_rotation);
	ROS_INFO("New displacement y :%g",angle_rotation);
	ROS_INFO("New target x:%g",angle_rotation);
	ROS_INFO("New target y:%g",angle_rotation);



    /*tf::TransformBroadcaster broadcaster;	
	tf::Transform transform,transform_2;
	tf::Quaternion q;
	q=tf::createQuaternionFromRPY(0,0,M_PI/3);
	transform.setOrigin(tf::Vector3(val,0.2,0));
	transform.setRotation(q);	

	//transform_2.setRotation(q);
	double new_x=transform.getOrigin().x()+0.1;
	double new_y=transform.getOrigin().y()-0.2;

	
	transform_2.setRotation(tf::Quaternion(0,0,0,1));
	transform_2.setOrigin(tf::Vector3(0.1,0.2,0));
	
	broadcaster.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"right_hand_camera","t2"));
		broadcaster.sendTransform(tf::StampedTransform(transform_2,ros::Time::now(),"t2","t3"));*/

}

int main(int argc,char**argv)
{
	ros::init(argc,argv,"baxter_broadcaster");
	ros::NodeHandle nh;

	dynamic_reconfigure::Server<baxter_test::rotationConfig> server;
	dynamic_reconfigure::Server<baxter_test::rotationConfig>::CallbackType f;

	
	tf::TransformBroadcaster broadcaster;

	tf::Transform marker_dummy_transform, target_transform;

	f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    ROS_INFO("Node spinning!");

    target_transform.setRotation(tf::Quaternion(0,0,0,1));

    while(nh.ok())
    {
    	marker_dummy_transform.setOrigin(tf::Vector3(displacement_x,displacement_y,0));
    	marker_dummy_transform.setRotation(tf::createQuaternionFromRPY(0,0,angle_rotation));

    	target_transform.setOrigin(tf::Vector3(target_x,target_y,0));

    	broadcaster.sendTransform(tf::StampedTransform(marker_dummy_transform,ros::Time::now(),"base",DUMMY_MARKER));
    	broadcaster.sendTransform(tf::StampedTransform(target_transform,ros::Time::now(),DUMMY_MARKER,TARGET));
    	sleep(0.1);
    	ros::spinOnce();

    }

    return 0;
}
