#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "stringpublisher");
    ros::NodeHandle n;
    ros::Publisher word_pub = n.advertise<std_msgs::String>("/words", 10);
    ros::Rate loop_rate(10);
    int count = 0;
    while (ros::ok())
    {
        std_msgs::String msg;
        stringstream ss;
        ss << "Hi sup' " << count;
        count++;

        msg.data = ss.str();

        cout << msg.data << endl;

        word_pub.publish(msg);
        loop_rate.sleep();
    }

    return 0;
}
