#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>

std::string turtle_name;

void poseCallback(const turtlesim::PoseConstPtr &msg)
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;

    transform.setOrigin(tf::Vector3(msg->x, msg->y, 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, msg->theta);
    transform.setRotation(q);

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", turtle_name));
}

void move(ros::NodeHandle n)
{
    ros::Publisher turtle_vel = n.advertise<geometry_msgs::Twist>(turtle_name + "/cmd_vel", 1000);
    ros::Rate loop_rate(10);

    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = 0.5;
    vel_msg.angular.z = 0.2;

    while (ros::ok())
    {
        turtle_vel.publish(vel_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_tf_broadcaster");
    ros::NodeHandle n;

    if (argc != 2)
    {
        ROS_ERROR("need turtle name as argument");
        return -1;
    }

    turtle_name = argv[1];

    ros::Subscriber sub = n.subscribe(turtle_name + "/pose", 10, &poseCallback);

    if (turtle_name == "/turtle1" || turtle_name == "turtle1")
    {
        move(n);
    }

    ros::spin();

    return 0;
}