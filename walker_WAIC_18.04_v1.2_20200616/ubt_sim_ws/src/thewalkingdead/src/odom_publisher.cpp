#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>

#include <iostream>
#include <mutex>

double camlink_x;
double camlink_y;
double camlink_z;

double odom_x;
double odom_y;
double odom_z;

double orb_x;
double orb_y;
double orb_z;
double orb_qx;
double orb_qy;
double orb_qz;
double orb_qw;

double imu_r;
double imu_p;
double imu_y;

double imu_vr;
double imu_vp;
double imu_vy;

double imu_ax;
double imu_ay;
double imu_az;

nav_msgs::Odometry walking_odom;

ros::Time last_tick_main, last_tick_imu;
double vx;
double vy;
double vz;

double dvx;
double dvy;
double dvz;

void orb_pose_callback(const geometry_msgs::PoseStamped &poseStamped)
{
    double last_orb_x = orb_x;
    double last_orb_y = orb_y;
    double last_orb_z = orb_z;

    orb_x = poseStamped.pose.position.x;
    orb_y = poseStamped.pose.position.y;
    orb_z = poseStamped.pose.position.z;

    orb_qx = poseStamped.pose.orientation.x;
    orb_qy = poseStamped.pose.orientation.y;
    orb_qz = poseStamped.pose.orientation.z;
    orb_qw = poseStamped.pose.orientation.w;

    // ros::Duration dt = ros::Time::now() - last_tick;
    // if (dt.toSec() < 1e-5)
    //     return;
    
    // double dx = orb_x - last_orb_x;
    // double dy = orb_y - last_orb_y;

    // std::cout << ros::Time::now() << " " << last_tick << std::endl;
    // std::cout << dx << " " << dy << " " << dt << std::endl;
    // vx = dx / dt.toSec();
    // vy = dy / dt.toSec();

    // last_tick = last_tick + dt;
}



void imu_callback(const sensor_msgs::Imu &imuData)
{
    imu_r = imuData.orientation.x;
    imu_p = imuData.orientation.y;
    imu_y = imuData.orientation.z;

    imu_vr = imuData.angular_velocity.x;
    imu_vp = imuData.angular_velocity.y;
    imu_vy = imuData.angular_velocity.z;

    imu_ax = imuData.linear_acceleration.x;
    imu_ay = imuData.linear_acceleration.y;
    imu_az = imuData.linear_acceleration.z;

    // double dt = (ros::Time::now() - last_tick_imu).getSec();
    // dvx += imu_ax * dt;
    // dvy += imu_ay * dt;

    // vx += dvx;
    // vy += dvy;
}

void walking_odom_callback(const nav_msgs::Odometry &w_odom_data)
{
    walking_odom = w_odom_data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odometry_publisher");

    ros::NodeHandle n;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50, true);
    tf::TransformBroadcaster odom_broadcaster;
    tf::TransformListener listener;

    vx = 0.0;
    vy = 0.0;
    vz = 0.0;

    dvx = 0.0;
    dvy = 0.0;
    dvz = 0.0;

    orb_x = 0.0;
    orb_y = 0.0;
    orb_z = 0.0;
    orb_qx = 0.0;
    orb_qy = 0.0;
    orb_qz = 0.0;
    orb_qw = 1.0;

    camlink_x = 0.0;
    camlink_y = 0.0;
    camlink_z = 0.0;

    imu_r = 0.0;
    imu_p = 0.0;
    imu_y = 0.0;

    imu_vr = 0.0;
    imu_vp = 0.0;
    imu_vy = 0.0;

    imu_ax = 0.0;
    imu_ay = 0.0;
    imu_az = 0.0;

    walking_odom = nav_msgs::Odometry();

    last_tick_main = ros::Time::now();
    last_tick_imu = ros::Time::now();

    ros::Subscriber orb_sub = n.subscribe("/orb_slam2_stereo/pose", 10, orb_pose_callback);
    // ros::Subscriber imu_sub = n.subscribe("/sensor/camera_imu", 10, imu_callback);
    // ros::Subscriber wodom_sub = n.subscribe("/Leg/walking_odom", 10, walking_odom_callback);

    // tf::StampedTransform last_camlink_transform;
    // last_camlink_transform.setData(tf::Transform::getIdentity());

    ros::Rate r(1000);
    ros::Time cur;
    while (n.ok())
    {
        cur = ros::Time::now();
        ros::spinOnce(); // check for incoming messages

        //For rotation, we will use IMU's orientation, since it's higher frequency.
        // tf::Matrix3x3 mat;
        // mat.setRPY(imu_r, imu_p, imu_y - 3.141592654 * 0.5);
        // tf::Quaternion odom_quat;
        // mat.getRotation(odom_quat);
        tf::StampedTransform map_camlink_transform;
        tf::StampedTransform map_odom_transform;
        try {
            listener.waitForTransform("/map", "/stereo_camera_link", ros::Time(0), ros::Duration(10.0));
            listener.lookupTransform("/map", "/stereo_camera_link", ros::Time(0), map_camlink_transform);
        }
        catch (tf::TransformException ex) {
            // ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        // try {
        //     listener.waitForTransform("/map", "/odom", ros::Time(0), ros::Duration(10.0));
        //     listener.lookupTransform("/map", "/odom", ros::Time(0), map_odom_transform);
        // }
        // catch (tf::TransformException ex) {
        //     ROS_ERROR("%s", ex.what());
        //     ros::Duration(1.0).sleep();
        //     continue;
        // }

        camlink_x = map_camlink_transform.getOrigin().getX();
        camlink_y = map_camlink_transform.getOrigin().getY();
        camlink_z = map_camlink_transform.getOrigin().getZ();

        // odom_x = map_odom_transform.getOrigin().getX();
        // odom_y = map_odom_transform.getOrigin().getY();
        // odom_z = map_odom_transform.getOrigin().getZ();

        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = cur;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = camlink_x;
        odom_trans.transform.translation.y = camlink_y;
        odom_trans.transform.translation.z = 0.0;
        quaternionTFToMsg(map_camlink_transform.getRotation(), odom_trans.transform.rotation);

        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

        // Provide a map/odom frame for cartographer, force to be static
        // geometry_msgs::TransformStamped map_trans; 
        // map_trans.header.stamp = cur;
        // map_trans.header.frame_id = "map";
        // map_trans.child_frame_id = "odom";
        // map_trans.transform.translation.x = 0.0;
        // map_trans.transform.translation.y = 0.0;
        // map_trans.transform.translation.z = 0.0;
        // quaternionTFToMsg(tf::Quaternion(0, 0, 0, 1), map_trans.transform.rotation);

        // odom_broadcaster.sendTransform(map_trans);

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = cur;
        odom.header.frame_id = "odom";

        //set the position, using data from visual odom
        odom.pose.pose.position.x = camlink_x;
        odom.pose.pose.position.y = camlink_y;
        odom.pose.pose.position.z = 0.0;
        quaternionTFToMsg(map_camlink_transform.getRotation(), odom.pose.pose.orientation);

        // double dt = (cur - last_tick_main).toSec();
        // if (dt > 1e-5)
        // {
        //     vx = (camlink_x - last_camlink_transform.getOrigin().getX()) / dt;
        //     vy = (camlink_y - last_camlink_transform.getOrigin().getY()) / dt;
        // }
        // last_tick_main = cur;
        // last_camlink_transform = map_camlink_transform;

        // //set the velocity, using data from IMU integral angular_v readout
        // odom.child_frame_id = "base_link";
        // odom.twist.twist.linear.x = vx;
        // odom.twist.twist.linear.y = vy;
        // odom.twist.twist.angular.z = imu_vy;

        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = 0.0;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.angular.z = 0.0;

        //publish the message
        odom_pub.publish(odom);
        
        r.sleep();
    }
}