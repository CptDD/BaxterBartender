#include <iostream>
#include <ros/ros.h>
#include <baxter_image_publisher/image_pub.h>
#include <ros/package.h>

#define NODE_NAME "baxter_image_publisher"
#define SERVICE_NAME "image_publisher_service"

using namespace std;


void publish_photo(ros::ServiceClient &client,string path,int number,ros::NodeHandle &nh)
{
	baxter_image_publisher::image_pub srv;

	stringstream ss;
	ss<<path<<"/images/"<<number<<".png";

	string img=ss.str();

	cout<<"The path is :"<<ss.str()<<endl<<img<<endl;

	srv.request.name.data=img;

	if(client.call(srv))
	{
		cout<<"Image published!"<<endl;
		nh.setParam("/button_pressed",false);
	}else
	{
		cout<<"An error has occured while publishing the image!"<<endl;
	}

}



int main(int argc,char**argv)
{
	ros::init(argc,argv,NODE_NAME);
	ros::NodeHandle nh;

	string path = ros::package::getPath("baxter_image_publisher");

	ros::ServiceClient client=nh.serviceClient<baxter_image_publisher::image_pub>(SERVICE_NAME);

	bool button;

	cout<<"Ok ?"<<endl;
	do
	{	
		nh.getParam("/button_pressed",button);

	}while(!button);

	publish_photo(client,path,0,nh);

	cout<<"Ok ?"<<endl;
	do
	{
		nh.getParam("/button_pressed",button);

	}while(!button);

	publish_photo(client,path,3,nh);

	cout<<"Ok ?"<<endl;
	do
	{
		nh.getParam("/button_pressed",button);

	}while(!button);

	publish_photo(client,path,2,nh);


	return 0;

}