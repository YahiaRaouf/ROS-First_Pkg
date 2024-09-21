#include "ros/ros.h"
#include "first_pkg/first_srv.h"
#include <iostream>
#include <sstream>

using namespace std;

int main(int argc , char **argv){
    ros::init(argc , argv , "Client");
    ros::NodeHandle nh;	//nodeHandle mainly starts and ends the node using ros::start() and ros::Shutdown()
    ros::Rate loop_rate(10);
    ros::ServiceClient client = nh.serviceClient<first_pkg::first_srv>("service");
    /*Creating a service client. this method is called handle service calling.
    there is another method called bare service calling , check roswiki*/

    while(ros::ok()){ // ros::ok() is what detects if the node is still active or not , and returns false once the node shutsdown 
        first_pkg::first_srv srv;
        /*creating a new service object instance*/

        std::stringstream ss;
        ss << "Sending from here";
        srv.request.in = ss.str();
        /*Filling request instance with a string "Sending From Here"*/

        if(client.call(srv)){
            ROS_INFO("From Client [%s] , server says [%s]" 
            , srv.request.in.c_str() , srv.response.out.c_str());
        }/*This will send the service call to the server ,
           if sent successfully , will print the request and the response*/

        else{
            ROS_ERROR("Failed to call service");
            return 1;
        }// this will happent if it fails

        ros::spinOnce();	
        loop_rate.sleep();
    }
    return 0;
}
