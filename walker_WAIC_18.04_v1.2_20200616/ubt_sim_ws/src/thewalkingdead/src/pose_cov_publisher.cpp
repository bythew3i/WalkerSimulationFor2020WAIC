#include <ros/ros.h>


#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

ros::Publisher pocov_pub;

void pose_msg_callback(const geometry_msgs::PoseStamped& poseStamped)
{
    geometry_msgs::PoseWithCovarianceStamped pocov;
    
    pocov.header = poseStamped.header;
    pocov.pose.pose = poseStamped.pose;

    double cov[36] =    {   0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.1, 0.0, 0.0, 0.0, 0.0,   
                            0.0, 0.0, 0.1, 0.0, 0.0, 0.0,   
                            0.0, 0.0, 0.0, 0.1, 0.0, 0.0,   
                            0.0, 0.0, 0.0, 0.0, 0.1, 0.0,   
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.1    };
    for (int i = 0; i < 36; i++)
    {
        pocov.pose.covariance[i] = cov[i];
    }
    pocov_pub.publish(pocov);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "orb_pose_with_covariance_pulisher");
    
    ros::NodeHandle n;
    pocov_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/orb_slam2_stereo/poseWithCovariance", 30);

    ros::Subscriber s = n.subscribe("/orb_slam2_stereo/pose", 30, pose_msg_callback);

    ROS_INFO("Sending pose msg with covariance...");
    ros::spin();
}