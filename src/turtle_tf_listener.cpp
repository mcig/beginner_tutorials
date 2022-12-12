#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_tf_listener");
    ros::NodeHandle node;

    ros::service::waitForService("spawn");
    ros::ServiceClient spawnClient = node.serviceClient<turtlesim::Spawn>("spawn");

    // Create the request and response objects .
    turtlesim::Spawn::Request req;
    turtlesim::Spawn::Response resp;
    req.x = 2;
    req.y = 3;
    req.theta = M_PI / 2;
    req.name = "/turtle2";
    bool success = spawnClient.call(req, resp);

    ros::Publisher turtle_vel = node.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);
    tf::TransformListener listener;
    ros::Rate rate(10.0);

    while (node.ok())
    {
        tf::StampedTransform transform;
        try
        {
            listener.waitForTransform("/turtle2", "/turtle1", ros::Time(0), ros::Duration(3.0));
            listener.lookupTransform("/turtle2", "/turtle1", ros::Time(0), transform);
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        geometry_msgs::Twist vel_msg;
        vel_msg.angular.z = 4.0 * (atan2(transform.getOrigin().y(), transform.getOrigin().x()));
        vel_msg.linear.x = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) + pow(transform.getOrigin().y(), 2));
        turtle_vel.publish(vel_msg);

        rate.sleep();
    }

    return 0;
}
