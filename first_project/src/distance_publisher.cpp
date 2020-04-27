#include <conversion.h>
#include "first_project/distance_calculate.h"
#include "first_project/dist.h"
#include <dynamic_reconfigure/server.h>
#include <first_project/flagConfig.h>

double safe;
double crash;

void callback(first_project::flagConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %lf %lf", 
            config.crash,config.safe);
            safe = config.safe;
            crash = config.crash;
            ROS_INFO ("%d",level);
}

int main(int argc, char **argv)
{
 ros::init(argc, argv, "subscribe_and_publish");
 ros::NodeHandle n;
 ros::ServiceClient client = n.serviceClient<first_project::distance_calculate>("distance_calculator");
 ros::Publisher dist_pub = n.advertise<first_project::dist>("distance", 1000);
 //conversion conversion_obs("/swiftnav/obs/gps_pose", "obs");
 //conversion conversion_front("/swiftnav/front/gps_pose", "front");

 n.getParam("/safe",safe);
 n.getParam("/crash",crash);

 dynamic_reconfigure::Server<first_project::flagConfig> server;
 dynamic_reconfigure::Server<first_project::flagConfig>::CallbackType f;

 f = boost::bind(&callback, _1, _2);
 server.setCallback(f);

 ros::Rate loop_rate(10);

 
 first_project::distance_calculate distance;

while(ros::ok()){
 
 if(client.call(distance)){
     first_project::dist msg;
     msg.header.stamp = ros::Time::now();
     msg.header.frame_id = "world";
     msg.dist = (float) distance.response.dist;

     if(msg.dist > crash && msg.dist < safe)
        msg.flag = "UnSafe";
     else if(msg.dist >= safe)
        msg.flag = "Safe";
     else
        msg.flag = "Crash";
     
     dist_pub.publish(msg); //inserire anche gli header nel message
}
 else{
	ROS_INFO("i valori non vanno bene");
 }

 ros::spinOnce();
 loop_rate.sleep();
}
 return 0;
}

