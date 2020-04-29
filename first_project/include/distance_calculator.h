#ifndef PROJECT_DISTANCE_H
#define PROJECT_DISTANCE_H

#include "ros/ros.h"
#include <math.h>
#include <nav_msgs/Odometry.h>
#include "first_project/distance_calculate.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

class distance_calculator{
public:

	distance_calculator();

        void calculateDistance(const nav_msgs::OdometryConstPtr& msg1, const nav_msgs::OdometryConstPtr& msg2);
	bool serverCallBack(first_project::distance_calculate::Request &req, first_project::distance_calculate::Response &res);

private:
	ros::NodeHandle n;
	ros::ServiceServer service;
	float distance = NAN;
	message_filters::Subscriber<nav_msgs::Odometry> subFront;
	message_filters::Subscriber<nav_msgs::Odometry> subObs;
	typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry> syncPolicy;
	message_filters::Synchronizer<syncPolicy> sync;
	

	void subscribeToNodes();
	void startServer();
};

#endif
