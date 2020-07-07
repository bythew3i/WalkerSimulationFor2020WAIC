#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>

#include <iostream>
#include <mutex>

ros::Publisher odom_pub;

// A plain relay
void odom_filtered_callback(const nav_msgs::Odometry& odom_filtered)
{    
    tf::TransformBroadcaster odom_broadcaster;
    odom_pub.publish(odom_filtered);

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = odom_filtered.header.stamp;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    
    odom_trans.transform.translation.x = odom_filtered.pose.pose.position.x;
    odom_trans.transform.translation.y = odom_filtered.pose.pose.position.y;
    odom_trans.transform.translation.z = odom_filtered.pose.pose.position.z;
    odom_trans.transform.rotation = odom_filtered.pose.pose.orientation;
    
    odom_broadcaster.sendTransform(odom_trans);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odometry_publisher");

    ros::NodeHandle n;
    odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50, true);
    
    ros::Subscriber s = n.subscribe("/odometry/filtered", 50, odom_filtered_callback);

    ROS_INFO("Broadcasting /odom topic and odom->base_link tf.");
    ros::spin();

    return 0;
}