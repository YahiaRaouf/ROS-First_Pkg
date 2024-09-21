#include "ros/ros.h"
#include "first_pkg/first_srv.h"        //generated header with service definition
#include <iostream>         
#include <sstream>      //string streaming class
using namespace std;

bool first_service_callback(first_pkg::first_srv::Request &req 
, first_pkg::first_srv::Response &res){

    std::stringstream ss;
    ss << "Received here";
    res.out = ss.str(); 
    /*out is the field name of the response given in service 
    file which will go to the client*/

    ROS_INFO("From Client [%s] , Server says [%s]" , req.in.c_str() , res.out.c_str());
    return true;
}

/*This is the server callback function executed 
when a request is received on the server. The
server can receive the request from clients with a message type of 
mastering_ros_demo_pkg::demo_srv::Request 
and sends the response in the
mastering_ros_demo_pkg::demo_srv::Response type*/

int main(int argc , char **argv){
    ros::init (argc , argv , "Server"); // intializing the node server
    ros::NodeHandle nh;     //creating a nodehandler object
    ros::ServiceServer service = nh.advertiseService("service" , first_service_callback);      
    /*Creating a Service and a callback function is executed when a 
    request is received from the client*/

    ROS_INFO("ready to receive from client");
    ros::spin();
    return 0;
}
