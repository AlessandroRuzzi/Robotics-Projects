#include "ros/ros.h"
#include <math.h>
#include <nav_msgs/Odometry.h>
#include "first_project/distance_calculate.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

float distanza;
bool isObsMoving = false;

void distance(const nav_msgs::OdometryConstPtr& msg1, const nav_msgs::OdometryConstPtr& msg2){
  isObsMoving = true;
  float x1 = msg1->pose.pose.position.x;
  float y1 = msg1->pose.pose.position.y;
  float z1 = msg1->pose.pose.position.z;
  float x2 = msg2->pose.pose.position.x;
  float y2 = msg2->pose.pose.position.y;
  float z2 = msg2->pose.pose.position.z;

 // ROS_INFO("x1,y1,z1 ->%f,%f,%f || x2,y2,z2 ->%f,%f,%f",x1,y1,z1,x2,y2,z2);
  distanza = sqrt(((x1 - x2) * (x1 - x2)) + ((y1 - y2) * (y1 - y2)) + ((z1 - z2) * (z1 - z2)));
  ROS_INFO("DIST: %f",distanza);
}

bool calculate(first_project::distance_calculate::Request &req, first_project::distance_calculate::Response &res)
{ 
  if(isObsMoving)
    res.dist = distanza;
  else
    res.dist = NAN;    

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "distance_calculator_server");
  ros::NodeHandle n;
  message_filters::Subscriber<nav_msgs::Odometry> sub1(n, "front", 1);
  message_filters::Subscriber<nav_msgs::Odometry> sub2(n, "obs", 1);

  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub1, sub2);
  sync.registerCallback(boost::bind(&distance, _1, _2));

  ros::ServiceServer service = n.advertiseService("distance_calculator",&calculate);
  ros::Rate loop_rate(10);

  ros::spin();

  return 0;
}
