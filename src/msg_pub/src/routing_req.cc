#include <ros/ros.h>

#include "routing_msgs/LaneWaypoint.h"
#include "routing_msgs/RosRoutingRequest.h"


int main(int argc, char **argv)
{
  //ROS节点初始化，告诉ROS Master
  ros::init(argc,argv,"routing_req");//定义节点名字
  
  //创建节点句柄
  ros::NodeHandle n;
  
  //创建一个Publisher，向名为/turtle1/cmd_vel的topic中发布message，
  //message的类型为geometry_msgs::Twist，队列的长度为10
  //队列的作用，发送方频率快，接收方处理频率慢，先将数据存放在缓冲区内
  ros::Publisher routing_req_pub=
      n.advertise<routing_msgs::RosRoutingRequest>("/routingrequest",10);

  //设置循环的频率
//  ros::Rate loop_rate(100000);

//  int count=0;
//  while(ros::ok())
//  {
    //初始化geometry_msgs::Twist类型的消息
    //geometry_msgs::Twist vel_msg;
    //vel_msg.linear.x=0.5;
    //vel_msg.angular.z=0.2;
    

    routing_msgs::RosRoutingRequest rrr;
    routing_msgs::LaneWaypoint lwp0;
    //lwp0.s=0;
    lwp0.x=8.156462219e+00;
    lwp0.y=3.980915673e+01;
    lwp0.z=0;
    rrr.waypoint.push_back(lwp0);

    routing_msgs::LaneWaypoint lwp1;
    //lwp1.s=;
    lwp1.x=8.150733483e+00;
    lwp1.y=3.980906431e+01;
    lwp1.z=0;
    rrr.waypoint.push_back(lwp1);    

    //发布消息
    while (routing_req_pub.getNumSubscribers()<1);

    
    routing_req_pub.publish(rrr);
    ROS_INFO("publish routing_msgs0 command [%s] [%f] [%f] [%f] ",
             rrr.waypoint[0].s,
             rrr.waypoint[0].x,
             rrr.waypoint[0].y,
             rrr.waypoint[0].z);
    ROS_INFO("publish routing_msgs1 command [%s] [%f] [%f] [%f] ",
             rrr.waypoint[1].s,
             rrr.waypoint[1].x,
             rrr.waypoint[1].y,
             rrr.waypoint[1].z);   
    
    //按照循环频率延时
//    loop_rate.sleep();
//  }
}