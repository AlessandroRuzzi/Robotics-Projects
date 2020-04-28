#ifndef PROJECT_DISTANCE_H
#define PROJECT_DISTANCE_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/NavSatFix.h"
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>  

class converter{
public:

	converter(std::string path,std::string object);

	void chatterCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
       
	void initParam(std::string pubTopic);
	void startPubAndSub(std::string path);

private:

	ros::NodeHandle n;
  	tf::TransformBroadcaster br;
  	ros::Subscriber sub;
	ros::Publisher pub_conv;
	ros::Time timeCallBack;
 	float latitude_init, longitude_init, h0;
  	float xEast, yNorth, zUp;
  	std::string pubTopic;

	void checkIfMessageIsPublished();
	void broadcastTf();
	void publishOdom();
	void publishNan();
};

#endif