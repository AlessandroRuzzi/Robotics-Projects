#include <distance_publisher.h>

distance_publisher::distance_publisher(){
  startClient();
  initParam();
  startDynamicReconfigure(); 
  startPublishing();
}


void distance_publisher::startClient(){
  client = n.serviceClient<first_project::distance_calculate>("distance_calculator");
  distPub = n.advertise<first_project::dist>("distance", 1000);
}

void distance_publisher::initParam(){
  n.getParam("/minSafe",minSafe);
  n.getParam("/minUnSafe",minUnSafe);
  n.getParam("/minCrash",minCrash);
}

void distance_publisher::startDynamicReconfigure(){
  f = boost::bind(&distance_publisher::reconfigureCallback,this, _1, _2);
  server.setCallback(f);
}

void distance_publisher::startPublishing(){

  ros::Rate loop_rate(10);
  while(ros::ok()){
    if(client.call(distance)){
      first_project::dist msg;
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = "world";
      msg.dist = (float) distance.response.dist;

      if(msg.dist > minUnSafe && msg.dist < minSafe)
        msg.flag = "UnSafe";
      else if(msg.dist >= minSafe)
        msg.flag = "Safe";
      else if(msg.dist >= minCrash && msg.dist <= minUnSafe)
        msg.flag = "Crash";
     
      distPub.publish(msg);
    }
    else{
      ROS_INFO("Server Error");
    }
  loop_rate.sleep();
  ros::spinOnce(); 
  }
}

void distance_publisher::reconfigureCallback(first_project::flagConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request(min UnSafe, min Safe): %lf %lf", 
            config.minUnSafe,config.minSafe);
            minSafe = config.minSafe;
            minUnSafe = config.minUnSafe;
            ROS_INFO ("%d",level);
}



int main(int argc, char **argv)
{
   ros::init(argc, argv, "distance_publisher");

   distance_publisher distance_publisher;

   ros::spin();
 
   return 0;
}



