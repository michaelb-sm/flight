/**
 * @file gps_node.cpp
 */

#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/HilGPS.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

nav_msgs::Odometry current_odom;
void camera_cb(const nav_msgs::Odometry::ConstPtr& msg){
    current_odom = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gps_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber cam_odom_sub = nh.subscribe<nav_msgs::Odometry>
            ("camera/odom/sample", 10, camera_cb);
    ros::Publisher fake_gps_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/fake_gps/vision", 10);
    ros::Publisher hil_gps_pub = nh.advertise<mavros_msgs::HilGPS>
            ("/mavros/hil/gps", 10);
    
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(5.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped fake_gps_pose;
    fake_gps_pose.header.frame_id = "map";
    // fake_gps_pose.pose.position.x = 0;
    // fake_gps_pose.pose.position.y = 0;
    // fake_gps_pose.pose.position.z = 0;
    // fake_gps_pose.pose.orientation.x = 0;
    // fake_gps_pose.pose.orientation.y = 0;
    // fake_gps_pose.pose.orientation.z = 0;
    // fake_gps_pose.pose.orientation.w = 1;


    mavros_msgs::HilGPS hil_gps;
    hil_gps.header.frame_id = "map";
    hil_gps.geo.latitude = 43.663360;
    hil_gps.geo.longitude = -79.393587;
    hil_gps.geo.altitude = 97.336;

    int count = 1;

    while(ros::ok())
    {
        fake_gps_pose.header.stamp = ros::Time::now();
        fake_gps_pose.header.seq = count;
        fake_gps_pose.pose = current_odom.pose.pose;
        fake_gps_pub.publish(fake_gps_pose);

        // hil_gps.header.stamp = ros::Time::now();
        // hil_gps.header.seq = count;
        // hil_gps_pub.publish(hil_gps);

        ros::spinOnce();
        count++;
        rate.sleep();
    }

    return 0;
}
