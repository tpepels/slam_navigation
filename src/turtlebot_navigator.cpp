#include "ros/ros.h"
#include "ros/console.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
// For transform support
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "tf/transform_datatypes.h"
#include "sensor_msgs/PointCloud.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"
#include <cstdlib> // Needed for rand()
#include <ctime> // Needed to seed random number generator with a time value

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
class TurtlebotNavigator {

public:
	// Construst a new RandomWalk object and hook up this ROS node
	// to the simulated robot's velocity control and laser topics
	TurtlebotNavigator(ros::NodeHandle& nh, tf::TransformListener& list) {
		// Initialize random time generator
		srand(time(NULL));
		tfListener = &list;
		frontierSub = nh.subscribe("frontiers", 1, &TurtlebotNavigator::frontierCallback, this);
	}
	;

	float getDistance(float x1, float x2, float y1, float y2)
	{
		return sqrt(pow((x1-x2), 2.) + pow((y1 - y2), 2.));
	}

	void frontierCallback( const sensor_msgs::PointCloud frontier_cloud )
	{
		//ROS_INFO("Frontier callback!");
		tf::StampedTransform transform;
		tfListener->waitForTransform("/map", "/odom", ros::Time(0), ros::Duration(3.0));
		tfListener->lookupTransform("/map", "/odom", ros::Time(0), transform);
		//		
		int frontier_i = 0;
		float closest_frontier_distance = 100000, distance = 0;
		//ROS_INFO("Navigation got %d frontiers", frontier_cloud.points.size());
		if(frontier_cloud.points.size() == 0)
			return;
		//
		for(int i = 0; i < frontier_cloud.points.size(); i++) {
			//
			distance = getDistance(frontier_cloud.points[i].x, transform.getOrigin().x(),
				frontier_cloud.points[i].y, transform.getOrigin().y());
			//
			if(distance > .7 && distance <= closest_frontier_distance) {
				closest_frontier_distance = distance;
				frontier_i = i;
			}
		}
		//
		ROS_INFO("Closest distance: %f", closest_frontier_distance);
		//
		move_base_msgs::MoveBaseGoal goal;
		goal.target_pose.header.frame_id = "map";
		goal.target_pose.header.stamp = ros::Time::now();
		//
		bool at_target = false;
		int attempts = 0;
		while(!at_target && attempts < 2) {
			//
			if(attempts >= 0){
				frontier_i = (rand() % frontier_cloud.points.size());
				at_target = false;
			}
			attempts++;

			// ROS_INFO("Frontier index: %d", frontier_i);			
			goal.target_pose.pose.position.x = frontier_cloud.points[frontier_i].x;
			goal.target_pose.pose.position.y = frontier_cloud.points[frontier_i].y;
			// Just give an orientation, since we are going to rotate in place at the destination
			geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0);
			goal.target_pose.pose.orientation = odom_quat;
			ROS_INFO("Navigating to: x: %f y: %f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
			//
			MoveBaseClient ac("move_base", true);
			//wait for the action server to come up
			while(!ac.waitForServer(ros::Duration(10.0))){
				ROS_INFO("Waiting for the move_base action server to come up");
			}
			
			ac.sendGoal(goal);
			ac.waitForResult(ros::Duration(45.0));

			if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
				at_target = true;
			  	ROS_INFO("The base moved to %f,%f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
			  	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(3.14);
				goal.target_pose.pose.orientation = odom_quat;
				ac.sendGoal(goal);
				ac.waitForResult();
			}
		}
	}
	;

	// Main FSM loop for ensuring that ROS messages are
	// processed in a timely manner, and also for sending
	// velocity controls to the simulated robot based on the FSM state
	void spin() {
		ros::Rate rate(50); // Specify the FSM loop rate in Hz
		while (ros::ok()) { // Keep spinning loop until user presses Ctrl+C
			ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
			rate.sleep(); // Sleep for the rest of the cycle, to enforce the FSM loop rate
		}
	}
	;
	// Tunable parameters

protected:
	ros::Subscriber frontierSub;
	tf::TransformListener *tfListener;
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "TurtlebotNavigator"); // Initiate new ROS node named "random_walk"
	ros::NodeHandle n;
	tf::TransformListener listener;
	TurtlebotNavigator walker(n, listener);
	ROS_INFO("Started!");
	walker.spin(); // Execute FSM loop
	return 0;
}
;
