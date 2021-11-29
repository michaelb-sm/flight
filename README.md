# flight
Temporary repository for drone work

## Running Code
Clone this repo under the catkin_ws/src folder, next to /mavros and /mavlink. Make sure to rebuild and source the catkin workspace. To run this code, you must first launch the mavros node:
> roslaunch mavros px4.launch fcu_url:=/dev/ttyTHS1:921600

To generate a fake GPS signal in lieu of a GPS module, run the gps_node.cpp file:
> rosrun flight gps_node

Main code lies in either the offb_node.cpp or flight_node.cpp file, which attempt to command a low velocity to the motors. These filesare run as such:
> rosrun flight offb_node
Or
> rosrun flight flight_node

## Code Breakdown
This code follows a similar structure to offb_node.cpp, found here:
> https://docs.px4.io/master/en/ros/mavros_offboard.html
