#include <ros/ros.h>
#include <string>
#include "routing_ros.h"

typedef apollo::routing::RoutingRos RoutingRos;
int main(int argc, char** argv)
{
    std::string NodeName="routing";
    ros::init(argc,argv,NodeName);
    RoutingRos* node= new RoutingRos();
    if(!(node->Init() && node->start()))
    {
        ROS_INFO("Fail to start node %s!",NodeName.c_str());
    }
    ros::Rate loop_rate(1000);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}