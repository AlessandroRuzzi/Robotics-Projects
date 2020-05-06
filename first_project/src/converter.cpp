#include <converter.h>

converter::converter(std::string path,std::string pubTopic){
  timeCallBack = ros::Time::now();
  initParam(pubTopic);
  startPubAndSub(path);
  timer = n.createTimer(ros::Duration(0.2), &converter::timerCallback, this);
}

void converter::initParam(std::string pubTopic){
  this->pubTopic = pubTopic;
  n.getParam("/longitude_init", longitude_init);
  n.getParam("/latitude_init", latitude_init);
  n.getParam("/h0", h0);
}

void converter::timerCallback(const ros::TimerEvent&){
	publishOdom(NAN,NAN,NAN);
}

void converter::startPubAndSub(std::string path){
  sub = n.subscribe(path, 1000, &converter::chatterCallback, this);
  pub_conv = n.advertise<nav_msgs::Odometry>(pubTopic, 1000);
}

void converter::chatterCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){
  timer.stop();
  if(msg->latitude == 0 && msg->longitude == 0 && msg->altitude == 0){
    publishOdom(NAN,NAN,NAN);             
    timer.start();
    return;
  }

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
  
  broadcastTf();
  publishOdom(xEast,yNorth,zUp);
  timer.start();

}

void converter::broadcastTf(){
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(xEast, yNorth, zUp) );
  tf::Quaternion q;
  q.setRPY(0, 0, 0);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", pubTopic)); 
 
}

void converter::publishOdom(float x,float y, float z){
  timeCallBack = ros::Time::now();
  nav_msgs::Odometry odom;
  odom.header.stamp = ros::Time::now();
  odom.header.frame_id = "world";
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = z;
  odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);

  pub_conv.publish(odom);
}

int main(int argc, char **argv){
  	
  ros::init(argc, argv,"converter");
  
  converter converter("/swiftnav/front/gps_pose",argv[1]);

  ros::spin();

  return 0;
}

