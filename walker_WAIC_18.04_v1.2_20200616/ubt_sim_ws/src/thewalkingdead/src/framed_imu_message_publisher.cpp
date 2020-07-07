#include <ros/ros.h>

#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Quaternion.h"

#include "tf/tf.h"

ros::Publisher boteye_imu_pub;
ros::Publisher orientus_imu_pub;
ros::Publisher head_imu_pub;

double head_imu_linear_acc_x_drift = 0.02663;
double orientus_imu_acc_x_drift = 0.02951;
double camera_imu_acc_x_drift = 0.02951;

void boteye_imu_msg_callback(const sensor_msgs::Imu &imumsg)
{
    sensor_msgs::Imu copy = imumsg;

    geometry_msgs::Quaternion rpy = copy.orientation;
    double r = rpy.x;
    double p = rpy.y;
    double y = rpy.z;
    geometry_msgs::Quaternion gq = tf::createQuaternionMsgFromRollPitchYaw(r, p, y);
    copy.orientation = gq;

    double orientation_cov[9] = {
        0.05, 0.0, 0.0,
        0.0, 0.05, 0.0,
        0.0, 0.0, 0.05
    };

    double angular_velocity_cov[9] = {
        0.05, 0.0, 0.0,
        0.0, 0.05, 0.0,
        0.0, 0.0, 0.05
    };

    double linear_acceleration_cov[9] = {
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0
    };

    for (int i = 0; i < 9; i++)
    {
        copy.orientation_covariance[i] = orientation_cov[i];
    }
    for (int i = 0; i < 9; i++)
    {
        copy.angular_velocity_covariance[i] = angular_velocity_cov[i];
    }
    for (int i = 0; i < 9; i++)
    {
        copy.linear_acceleration_covariance[i] = linear_acceleration_cov[i];
    }

    copy.linear_acceleration.x -= camera_imu_acc_x_drift;

    copy.header.frame_id = "boteye_imu_link";
    boteye_imu_pub.publish(copy);
}

void orientus_imu_msg_callback(const sensor_msgs::Imu &imumsg)
{
    sensor_msgs::Imu copy = imumsg;

    geometry_msgs::Quaternion rpy = copy.orientation;
    double r = rpy.x;
    double p = rpy.y;
    double y = rpy.z;
    geometry_msgs::Quaternion gq = tf::createQuaternionMsgFromRollPitchYaw(r, p, y);
    copy.orientation = gq;

    double orientation_cov[9] = {
        0.05, 0.0, 0.0,
        0.0, 0.05, 0.0,
        0.0, 0.0, 0.05
    };

    double angular_velocity_cov[9] = {
        0.05, 0.0, 0.0,
        0.0, 0.05, 0.0,
        0.0, 0.0, 0.05
    };

    double linear_acceleration_cov[9] = {
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0
    };

    for (int i = 0; i < 9; i++)
    {
        copy.orientation_covariance[i] = orientation_cov[i];
    }
    for (int i = 0; i < 9; i++)
    {
        copy.angular_velocity_covariance[i] = angular_velocity_cov[i];
    }
    for (int i = 0; i < 9; i++)
    {
        copy.linear_acceleration_covariance[i] = linear_acceleration_cov[i];
    }

    copy.linear_acceleration.x -= orientus_imu_acc_x_drift;

    copy.header.frame_id = "orientus_imu_link";
    orientus_imu_pub.publish(copy);
}

void head_imu_msg_callback(const sensor_msgs::Imu &imumsg)
{
    sensor_msgs::Imu copy = imumsg;

    geometry_msgs::Quaternion rpy = copy.orientation;
    double r = rpy.x;
    double p = rpy.y;
    double y = rpy.z;
    geometry_msgs::Quaternion gq = tf::createQuaternionMsgFromRollPitchYaw(r, p, y);
    copy.orientation = gq;

    double orientation_cov[9] = {
        0.05, 0.0, 0.0,
        0.0, 0.05, 0.0,
        0.0, 0.0, 0.05
    };

    double angular_velocity_cov[9] = {
        0.05, 0.0, 0.0,
        0.0, 0.05, 0.0,
        0.0, 0.0, 0.05
    };

    double linear_acceleration_cov[9] = {
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0
    };

    for (int i = 0; i < 9; i++)
    {
        copy.orientation_covariance[i] = orientation_cov[i];
    }
    for (int i = 0; i < 9; i++)
    {
        copy.angular_velocity_covariance[i] = angular_velocity_cov[i];
    }
    for (int i = 0; i < 9; i++)
    {
        copy.linear_acceleration_covariance[i] = linear_acceleration_cov[i];
    }

    copy.linear_acceleration.x -= head_imu_linear_acc_x_drift;

    copy.header.frame_id = "boteye_imu_link";
    head_imu_pub.publish(copy);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "framed_imu_message_publisher");
    
    ros::NodeHandle n;
    boteye_imu_pub = n.advertise<sensor_msgs::Imu>("/sensor/camera_imu_framed", 30);
    orientus_imu_pub = n.advertise<sensor_msgs::Imu>("/sensor/orientus_imu_framed", 30);
    head_imu_pub = n.advertise<sensor_msgs::Imu>("/sensor/head_imu_framed", 30);
    
    ros::Subscriber isub = n.subscribe("/sensor/camera_imu", 30, boteye_imu_msg_callback);
    ros::Subscriber isub2 = n.subscribe("/sensor/orientus_imu", 30, orientus_imu_msg_callback);
    ros::Subscriber isub3 = n.subscribe("/sensor/head_imu", 30, head_imu_msg_callback);

    ROS_INFO("Publishing framed imu...");
    ros::spin();
}