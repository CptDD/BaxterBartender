#include <iostream>
#include <ros/ros.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene/planning_scene.h>
#include <geometric_shapes/shape_operations.h>


using namespace std;


#define NODE_DESCRIPTION "A used to insert the collision objects to the scene in Rviz"
#define NODE_NAME "pick_objects"



int main(int argc,char**argv)
{
	ros::init(argc,argv,NODE_NAME);
	ros::NodeHandle node_handle;
		
	cout<<NODE_DESCRIPTION<<endl;

	ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
	while(planning_scene_diff_publisher.getNumSubscribers() < 1)	
	{
	  ros::WallDuration sleep_t(0.5);
	  sleep_t.sleep();
	}	

	Eigen::Vector3d scale(1,1,1);

	
	moveit_msgs::AttachedCollisionObject table_object;
	table_object.link_name="base";
	table_object.object.header.frame_id="base";
	table_object.object.id="table";

	geometry_msgs::Pose pose;
	pose.orientation.w = 1.0;
	pose.position.x=1.0;
	pose.position.y=0;
	pose.position.z=-0.17;

	shape_msgs::SolidPrimitive primitive;
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	primitive.dimensions[0] = 0.913;
	primitive.dimensions[1] = 0.913;
	primitive.dimensions[2] = 0.04;
	
	table_object.object.primitives.push_back(primitive);
	table_object.object.primitive_poses.push_back(pose);
	table_object.object.operation = table_object.object.ADD;
	
	moveit_msgs::PlanningScene planning_scene;
	planning_scene.is_diff = true;
	planning_scene.world.collision_objects.push_back(table_object.object);
	planning_scene_diff_publisher.publish(planning_scene);

	sleep(3.0);

	cout<<"Table primitive inserted!"<<endl;

	moveit_msgs::AttachedCollisionObject beer_bottle;


	shapes::Mesh *bottle_mesh=shapes::createMeshFromResource("package://baxter_test/meshes/beer_bottle_up_75.stl",scale);
	beer_bottle.link_name="base";
	beer_bottle.object.header.frame_id="base";
	beer_bottle.object.id="beer_bottle";

	shape_msgs::Mesh mesh;
   	shapes::ShapeMsg mesh_msg;  
    	shapes::constructMsgFromShape(bottle_mesh, mesh_msg);
    	mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

	beer_bottle.object.mesh_poses.resize(1);

	
	beer_bottle.object.mesh_poses[0].position.x = 0.912017;
   	beer_bottle.object.mesh_poses[0].position.y = -0.257126;
    	beer_bottle.object.mesh_poses[0].position.z = -0.017;
    	beer_bottle.object.mesh_poses[0].orientation.w= 0.0; 
    	beer_bottle.object.mesh_poses[0].orientation.x= 0.0; 
    	beer_bottle.object.mesh_poses[0].orientation.y= 0.0;
    	beer_bottle.object.mesh_poses[0].orientation.z= 0.0;
	
	beer_bottle.object.meshes.push_back(mesh);	
	beer_bottle.object.operation=beer_bottle.object.ADD;

	planning_scene.is_diff=true;
	planning_scene.world.collision_objects.push_back(beer_bottle.object);
        planning_scene_diff_publisher.publish(planning_scene);	

	sleep(3.0);
	cout<<"Beer bottle added!"<<endl;	

	moveit_msgs::AttachedCollisionObject beer_pint;
	
	shapes::Mesh* pint_mesh=shapes::createMeshFromResource("package://baxter_test/meshes/beer_pint_75.stl",scale);
	beer_pint.link_name="base";
	beer_pint.object.header.frame_id="base";
	beer_pint.object.id="beer_pint";

	shape_msgs::Mesh mesh_pint;
	shapes::ShapeMsg pint_mesh_msg;
	shapes::constructMsgFromShape(pint_mesh, pint_mesh_msg);
    	mesh_pint = boost::get<shape_msgs::Mesh>(pint_mesh_msg);
	
	
	beer_pint.object.mesh_poses.resize(1);
	beer_pint.object.mesh_poses[0].position.x=0.904686;
	beer_pint.object.mesh_poses[0].position.y=0.260688;
	beer_pint.object.mesh_poses[0].position.z=-0.017;
	beer_pint.object.mesh_poses[0].orientation.w=0.0;
	beer_pint.object.mesh_poses[0].orientation.x=0.0;
	beer_pint.object.mesh_poses[0].orientation.y=0.0;
	beer_pint.object.mesh_poses[0].orientation.z=0.0;
	
	beer_pint.object.meshes.push_back(mesh_pint);	
	beer_pint.object.operation=beer_bottle.object.ADD;
	
	planning_scene.is_diff=true;
	planning_scene.world.collision_objects.push_back(beer_pint.object);
        planning_scene_diff_publisher.publish(planning_scene);	

	sleep(3.0);
	cout<<"Beer pint added!"<<endl;	
	
	/*moveit_msgs::CollisionObject collision_object;
	collision_object.id = "beer_bottle_up.stl";
	collision_object.header.frame_id = "base";
	collision_object.operation = collision_object.REMOVE;
        planning_scene.world.collision_objects.push_back(collision_object);
	
	moveit_msgs::AttachedCollisionObject attached_object;
	attached_object.object.id="beer_bottle_up.stl";
        attached_object.object.header.frame_id = "right_gripper";
        attached_object.link_name = "right_gripper";
        attached_object.touch_links.push_back("right_gripper");


	attached_object.object.operation = attached_object.object.ADD;
    	planning_scene.robot_state.attached_collision_objects.push_back(attached_object);

    	planning_scene_diff_publisher.publish(planning_scene);
		
	sleep(5.0);
	cout<<"Done"<<endl;
	*/
	
	return 0;
	
}	
