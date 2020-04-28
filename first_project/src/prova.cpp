#include <distance_calculator.h>

distance_calculator::distance_calculator(){
  //subscribeToNodes();
  //startServer();
 message_filters::Subscriber<nav_msgs::Odometry> sub1(n, "front", 1);
  message_filters::Subscriber<nav_msgs::Odometry> sub2(n, "obs", 1);
  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub1, sub2);
  sync.registerCallback(boost::bind(&distance_calculator::calculateDistance,this, _1, _2));

service = n.advertiseService("distance_calculator",&distance_calculator::serverCallBack,this);
ROS_INFO("CIAOSER_VER");
}

void distance_calculator::subscribeToNodes(){
  message_filters::Subscriber<nav_msgs::Odometry> sub1(n, "front", 1);
  message_filters::Subscriber<nav_msgs::Odometry> sub2(n, "obs", 1);
  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub1, sub2);
  sync.registerCallback(boost::bind(&distance_calculator::calculateDistance,this, _1, _2));
ROS_INFO("CIAOSER_VER");
}

void distance_calculator::startServer(){
service = n.advertiseService("distance_calculator",&distance_calculator::serverCallBack,this);
ROS_INFO("CIAOSER_VER");
}

void distance_calculator::calculateDistance(const nav_msgs::OdometryConstPtr& msg1, const nav_msgs::OdometryConstPtr& msg2){
  
  float x1 = msg1->pose.pose.position.x;
  float y1 = msg1->pose.pose.position.y;
  float z1 = msg1->pose.pose.position.z;
  float x2 = msg2->pose.pose.position.x;
  float y2 = msg2->pose.pose.position.y;
  float z2 = msg2->pose.pose.position.z;
  distance = sqrt(((x1 - x2) * (x1 - x2)) + ((y1 - y2) * (y1 - y2)) + ((z1 - z2) * (z1 - z2)));

}

bool distance_calculator::serverCallBack(first_project::distance_calculate::Request &req, first_project::distance_calculate::Response &res){
  res.dist = distance;
  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "distance_calculator_server");
  
  distance_calculator distance_calculator1;
  
  ros::spin();

  return 0;
}
