//
// Created by dj on 2026/10/30.
//
#include <ros/ros.h>
#include <std_msgs/String.h>

void talkercb(std_msgs::String msg)
{
    ROS_INFO("I received the message: %s", msg.data.c_str());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker_listener");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<std_msgs::String>("talker", 100);
    ros::Subscriber sub = n.subscribe("talker", 100, &talkercb);
    std_msgs::String msg;
    msg.data = "Hello World!";
    ROS_INFO("I published the message: %s", msg.data.c_str());
    pub.publish(msg);
    ros::spin();
    return 0;
}