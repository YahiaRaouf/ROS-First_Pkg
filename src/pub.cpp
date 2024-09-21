#include "ros/ros.h"   //this header file is tha main header file for ROS to use roscpp client APIs
#include "std_msgs/String.h"
#include "first_pkg/custom_msg.h"   //this inludes the integer data type forom std_msgs
#include <iostream>

int main (int argc , char** argv) {

    ros::init(argc, argv,"pub");      //this line is what initializes a node , given the name in the third arg of the function init
    ros::NodeHandle node_obj;       //this line creates a nodehandle obj which is responsible for communicating with ros system
    ros::Publisher number_publisher = 
    node_obj.advertise<first_pkg::custom_msg>("/numbers",10); //this initialzizes a topic as a pub and given a name /numbers with a buffer size of 10
    ros::Rate loop_rate(1);    //set the frequency rate to 10

    int number_count = 0;

    while (ros::ok()){

        first_pkg::custom_msg msg;        //here we create a ros message of type int32
        msg.name.data= "Yahia";
        msg.number.data = number_count;        //and assign the vaiable number_count to the msg field "data"
        ROS_INFO("%s",msg.name.data.c_str());
        ROS_INFO("%d",msg.number.data);        //this is what displays the Ros message data in int format %d
        number_publisher.publish(msg);      //now here we ask the publisher number_publisher to publish the msg to the topic /numbers
        ros::spinOnce();        //this commmand is essential as it reads and updates all ROS topics , nodes wont publish without it , the is also spin()
        loop_rate.sleep();      //This line will provide the necessary delay to achieve a frequency of 10 Hz.
        ++number_count;
    }

return 0;

}
