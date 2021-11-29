/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geographic_msgs/GeoPoint.h>
#include <mavros_msgs/HilGPS.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher hil_gps_pub = nh.advertise<mavros_msgs::HilGPS>
            ("/mavros/hil/gps", 10);
    ros::Publisher velocity_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel", 10);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::HilGPS hil_gps;
    hil_gps.header.frame_id = 1;
    hil_gps.header.stamp = ros::Time::now();
    hil_gps.header.seq = 1;
    hil_gps.geo.latitude = 43.663360;
    hil_gps.geo.longitude = -79.393587;
    hil_gps.geo.altitude = 97.336;

    hil_gps_pub.publish(hil_gps);

    geometry_msgs::TwistStamped vel;
    vel.header.frame_id = 1;
    vel.twist.linear.x = 0;
    vel.twist.linear.y = 0;
    vel.twist.linear.z = 0.1;
    vel.twist.angular.x = 0;
    vel.twist.angular.y = 0;
    vel.twist.angular.z = 0;

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    //send a few setpoints before starting
    int count = 1;
    for(int i = 100; ros::ok() && i > 0; --i){
        vel.header.stamp = ros::Time::now();
        vel.header.seq = count;
        velocity_pub.publish(vel);
        ros::spinOnce();
        count++;
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO_STREAM("Offboard success?" << offb_set_mode.response.mode_sent);
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }  else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        vel.header.stamp = ros::Time::now();
        vel.header.seq = count;
        velocity_pub.publish(vel);

        ros::spinOnce();
        count++;
        rate.sleep();
    }

    return 0;
}