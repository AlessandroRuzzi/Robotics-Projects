#include <distance_calculator.h>

distance_calculator::distance_calculator():sync(syncPolicy(10), subFront, subObs){
  subscribeToNodes();
  startServer();
}

void distance_calculator::subscribeToNodes(){
  subFront.subscribe(n, "front", 1);
  subObs.subscribe(n, "obs", 1);
  sync.registerCallback(boost::bind(&distance_calculator::calculateDistance,this, _1, _2));
}

void distance_calculator::startServer(){
  service = n.advertiseService("distance_calculator",&distance_calculator::serverCallBack,this);
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
  
  distance_calculator distance_calculator;
  
  ros::spin();

  return 0;
}
