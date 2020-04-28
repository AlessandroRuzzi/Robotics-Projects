#include <converter.h>

converter::converter(std::string path,std::string pubTopic){
  timeCallBack = ros::Time::now();
  initParam(pubTopic);
  startPubAndSub(path);
  checkIfMessageIsPublished();
}

void converter::initParam(std::string pubTopic){
this->pubTopic = pubTopic;
  n.getParam("/longitude_init", longitude_init);
  n.getParam("/latitude_init", latitude_init);
  n.getParam("/h0", h0);
}

void converter::startPubAndSub(std::string path){
  sub = n.subscribe(path, 1000, &converter::chatterCallback, this);
  pub_conv = n.advertise<nav_msgs::Odometry>(pubTopic, 1000);
}

void converter::checkIfMessageIsPublished(){
  ros::Rate loop_rate(10);

  ros::Time lastCallBack = timeCallBack;
  while(ros::ok()){
   
   if(lastCallBack == timeCallBack)
       publishNan();

  lastCallBack = timeCallBack;
  loop_rate.sleep();
  ros::spinOnce();  //vedere se si può togliere
  }
}

void converter::chatterCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){
  //ROS_INFO("Input position: [%f,%f, %f]", msg->latitude, msg->longitude,msg->altitude);

  // fixed values

  double a = 6378137;
  double b = 6356752.3142;
  double f = (a - b) / a;
  double e_sq = f * (2-f);
  float deg_to_rad = 0.0174533;
  
  // input data from msg
  float latitude = msg->latitude;
  float longitude = msg->longitude;
  float h = msg->altitude;

  // fixed position
  
	

  //lla to ecef
  float lamb = deg_to_rad*(latitude);
  float phi = deg_to_rad*(longitude);
  float s = sin(lamb);
  float N = a / sqrt(1 - e_sq * s * s);

  float sin_lambda = sin(lamb);
  float  cos_lambda = cos(lamb);
  float  sin_phi = sin(phi);
  float  cos_phi = cos(phi);

  float  x = (h + N) * cos_lambda * cos_phi;
  float  y = (h + N) * cos_lambda * sin_phi;
  float  z = (h + (1 - e_sq) * N) * sin_lambda;
  
  //ROS_INFO("ECEF position: [%f,%f, %f]", x, y,z);
  

  // ecef to enu
 
  lamb = deg_to_rad*(latitude_init);
  phi = deg_to_rad*(longitude_init);
  s = sin(lamb);
  N = a / sqrt(1 - e_sq * s * s);

  sin_lambda = sin(lamb);
  cos_lambda = cos(lamb);
  sin_phi = sin(phi);
  cos_phi = cos(phi);

  float  x0 = (h0 + N) * cos_lambda * cos_phi;
  float  y0 = (h0 + N) * cos_lambda * sin_phi;
  float  z0 = (h0 + (1 - e_sq) * N) * sin_lambda;

  float  xd = x - x0;
  float  yd = y - y0;
  float  zd = z - z0;

  xEast = -sin_phi * xd + cos_phi * yd;
  yNorth = -cos_phi * sin_lambda * xd - sin_lambda * sin_phi * yd + cos_lambda * zd;
  zUp = cos_lambda * cos_phi * xd + cos_lambda * sin_phi * yd + sin_lambda * zd;

  //ROS_INFO("ENU position: [%f,%f, %f]", xEast, yNorth, zUp);
  
  broadcastTf();
  publishOdom();

}

void converter::broadcastTf(){
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(xEast/100, yNorth/100, zUp/100) );
  tf::Quaternion q;
  q.setRPY(0, 0, 0);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", pubTopic)); 
 
}

void converter::publishOdom(){
  timeCallBack = ros::Time::now();
  nav_msgs::Odometry odom;
  odom.header.stamp = ros::Time::now();
  odom.header.frame_id = "odom";
  odom.pose.pose.position.x = xEast;
  odom.pose.pose.position.y = yNorth;
  odom.pose.pose.position.z = zUp;
  odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);

  pub_conv.publish(odom);
}

void converter::publishNan(){
  nav_msgs::Odometry odom;
  odom.header.stamp = ros::Time::now();
  odom.header.frame_id = "odom";
  odom.pose.pose.position.x = NAN;
  odom.pose.pose.position.y = NAN;
  odom.pose.pose.position.z = NAN;
  odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);

  pub_conv.publish(odom);

}

int main(int argc, char **argv){
  	
  ros::init(argc, argv,"converter");

  converter converter1("/swiftnav/front/gps_pose",argv[1]);

  ros::spin();

  return 0;
}

