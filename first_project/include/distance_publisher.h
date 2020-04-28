#ifndef PROJECT_DISTANCE_A
#define PROJECT_DISTANCE_A

#include "first_project/distance_calculate.h"
#include "first_project/dist.h"
#include <dynamic_reconfigure/server.h>
#include <first_project/flagConfig.h>

class distance_publisher{
public:

	distance_publisher();
	void reconfigureCallback(first_project::flagConfig &config, uint32_t level);

	

private:

	ros::NodeHandle n;
	ros::ServiceClient client;
	ros::Publisher distPub;	
	dynamic_reconfigure::Server<first_project::flagConfig> server;
        dynamic_reconfigure::Server<first_project::flagConfig>::CallbackType f;
	double minSafe;
        double minUnSafe;
	double minCrash;
	first_project::distance_calculate distance;

	void startClient();
	void initParam();
        void startDynamicReconfigure();
	void startPublishing();
 
};

#endif
