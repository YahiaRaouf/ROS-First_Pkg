#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "first_pkg/custom_msg.h"
#include <iostream>

void callback (const first_pkg::custom_msg::ConstPtr& msg) {
ROS_INFO("Received [%s]",msg->name.data.c_str());
ROS_INFO("Received [%d]" , msg->number.data);
}
/*This is a callback function that will execute whenever a data comes to the /numbers
topic. Whenever a data reaches this topic, the function will call and extract the value and
print it on the console*/

int main(int argc , char** argv){
    ros::init(argc , argv , "sub");       // here we intialized the sub node
    ros:: NodeHandle node_obj;      //and created a nodehandle object to connect the node with the ros system
    ros:: Subscriber number_subscriber = node_obj.subscribe("/numbers" , 10 , callback);      //and here we gave the subscriber it name "number_subscriber" and connect it
                                                        // to the topic "/number" and use the call back function to react to each digit given to it
    ros::spin();     //This is an infinite loop in which the node will wait in this step. 
                    //This code will fasten the callbacks whenever a data reaches the topic. The node will quit only when we press the Ctrl + C key.
    return 0; 
}
