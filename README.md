# flight
Temporary repository for drone work

## Running Code
Clone this repo under the catkin_ws/src folder, next to /mavros and /mavlink. Make sure to rebuild and source the catkin workspace. To run this code, you must first launch the mavros node:
> roslaunch mavros px4.launch fcu_url:=/dev/ttyTHS1:921600
You can then run the offb_node.cpp file with:
> rosrun flight offb_node

## Code Breakdown
This code follows a similar structure to offb_node.cpp, found here:
> https://docs.px4.io/master/en/ros/mavros_offboard.html
