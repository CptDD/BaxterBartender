#include <iostream>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>

using namespace std;


void callback(const ar_track_alvar_msgs::AlvarMarkers &msg)
{
	cout<<"Message has been received!"<<endl;

	static tf::TransformBroadcaster broadcaster;


	for(int i=0;i<msg.markers.size();i++)
	{
		ar_track_alvar_msgs::AlvarMarker marker=msg.markers[i];
		geometry_msgs::Pose pose=marker.pose.pose;

		tf::Transform transform;


		/*transform.setOrigin(tf::Vector3(pose.position.x,pose.position.y,pose.position.z));
		transform.setRotation(tf::Quaternion(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w));*/

		if(marker.id==9)
		{
			transform.setOrigin(tf::Vector3(0.08,0.09,0));
			transform.setRotation(tf::Quaternion(0,0,0,1));

			broadcaster.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"/ar_marker_9","/golfer"));
		}
		cout<<marker.id<<endl;
	}
}



int main(int argc,char **argv)
{
	ros::init(argc,argv,"marker_offset");
	ros::NodeHandle nh;


	ros::Subscriber subscriber=nh.subscribe("/ar_pose_marker",10,&callback);

	ros::spin();
	return 0;

}