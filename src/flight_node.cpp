/**
 * @file flight_node.cpp
 */

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandHome.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "flight_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher velocity_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel", 10);
    ros::ServiceClient set_hp_client = nh.serviceClient<mavros_msgs::CommandHome>
            ("mavros/cmd/set_home");
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

    //set home position as current position
    mavros_msgs::CommandHome set_hp_cmd;
    set_hp_cmd.request.current_gps = true;
    while(!set_hp_client.call(set_hp_cmd) && 
            set_hp_cmd.response.success){

        ROS_INFO("Failed to set home");
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("HP set");

    geometry_msgs::TwistStamped vel;
    vel.header.frame_id = 1;
    vel.twist.linear.x = 0;
    vel.twist.linear.y = 0;
    vel.twist.linear.z = 0.1;
    vel.twist.angular.x = 0;
    vel.twist.angular.y = 0;
    vel.twist.angular.z = 0;

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