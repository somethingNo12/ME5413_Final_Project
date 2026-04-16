#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "initial_pose_publisher");
    ros::NodeHandle nh;


    ros::Publisher pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, true);


    ros::Duration(1.0).sleep();

    geometry_msgs::PoseWithCovarianceStamped msg;

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";

    //set the initial pose
    msg.pose.pose.position.x = 0.0;
    msg.pose.pose.position.y = 0.0;
    msg.pose.pose.position.z = 0.0;

    // orietation
    msg.pose.pose.orientation.x = 0.0;
    msg.pose.pose.orientation.y = 0.0;
    msg.pose.pose.orientation.z = 0.0;
    msg.pose.pose.orientation.w = 1.0;

    for(int i = 0; i < 36; i++)
        msg.pose.covariance[i] = 0.0;

    msg.pose.covariance[0] = 0.25;   // x
    msg.pose.covariance[7] = 0.25;   // y
    msg.pose.covariance[35] = 0.1;   // yaw

    ROS_INFO("Publishing initial pose...");

    pub.publish(msg);

    ros::spinOnce();

    return 0;
}
