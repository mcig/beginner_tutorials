#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

using namespace std;

void callback(const std_msgs::String::ConstPtr &msg)
{
    ROS_INFO("heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "stringsubscriber");
    ros::NodeHandle n;
    ros::Subscriber word_sub = n.subscribe<std_msgs::String>("/words", 10, &callback);

    ros::spin();

    return 0;
}