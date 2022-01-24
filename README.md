# flight
Temporary repository for drone work

## Running Code
Clone this repo under the catkin_ws/src folder, next to /mavros and /mavlink. Make sure to rebuild and source the catkin workspace. 

### The MAVROS Node

To run this code, you must first launch the mavros node, which acts as the main communications bridge between ROS and the *Pixhawk 4 Mini* using the **MAVLink** protocol:

```shell
$ roslaunch mavros px4.launch fcu_url:=/dev/ttyTHS1:921600
```

The fcu address is specified here as ``/dev/ttyTHS1``, which directs communications over the GPIO pins of the *Jetson Nano* onboard computer. The baudrate is specified as ``921600``, which is the recommended rate for fast communication.

### Realsense & GPS (Position) Nodes

To generate a position signal for the Pixhawk in lieu of a GPS module, you must first start a node to generate data from the onboard *Realsense T265* stereo camera:

```shell
$ roslaunch realsense2_camera rs_t265.launch
```

More about installing and configuring the T265 can be found under ``ROB498-flight/docs/realsense_t265.md``. The T265 camera performs its own onboard VSLAM which generates a position estimate that can be read over ROS.

Then, run the gps_node.cpp file:

```shell
$ rosrun flight gps_node
```

This reads data from the streamed realsense2_camera node (``camera/odom/sample``) and reformats it into a position estimate that the Pixhawk will be able to read. NOTE THAT THE ESTIMATE IS CURRENTLY SENT IN THE "ODOM" FRAME, WHICH IS A PRECONFIGURED PIXHAWK FRAME. IN PRACTICE, A PROPER CAMERA -> PIXHAWK TRANSFORM SHOULD BE INTRODUCED (not investigated thus far).

### The Flight Node

Main code lies in the flight_node.cpp file, which attempts to command a low velocity to the motors. This file is run with:

```shell
$ rosrun flight flight_node
```

See the below link for a much more indepth breakdown of the code. In shallow terms, this node sends a constant stream of motor commands to the Pixhawk before arming the drone, after which it starts to accept and execute the given commands.

#### Code Breakdown
This code follows a similar structure to offb_node.cpp, found here:
> https://docs.px4.io/master/en/ros/mavros_offboard.html

## Contact
For questions about this code, SPECIFICALLY FOR DEVELOPMENT OF THE ROB498 ROBOTICS CAPSTONE COURSE AT THE UNIVERSITY OF TORONTO, you can contact Michael Shanks-Marshall at michael.shanks.marshall@mail.utoronto.ca
