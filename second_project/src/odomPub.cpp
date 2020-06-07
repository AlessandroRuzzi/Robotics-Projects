#include "ros/ros.h"
#include <nav_msgs/Odometry.h>  


class pub_sub
{

private:
ros::NodeHandle n; 

ros::Subscriber sub;
ros::Publisher pub; 
	
	
public:
  	pub_sub(){
  	sub = n.subscribe("/camera/odom/sample", 1, &pub_sub::callback, this);
	pub = n.advertise<nav_msgs::Odometry>("/vo", 1);
	
}
void callback(const nav_msgs::Odometry::ConstPtr& msg){

  pub.publish(msg);

}


};

int main(int argc, char **argv)
{
 	ros::init(argc, argv, "subscribe_and_publish");
 	pub_sub my_pub_sub;
 	ros::spin();
 	return 0;
}
