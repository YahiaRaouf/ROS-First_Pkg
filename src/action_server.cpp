#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <actionlib/server/simple_action_server.h> /*this is a standard acrtion library to implement
an action server*/
#include "first_pkg/first_actAction.h"             /*this includes accessing our action definition in action folder*/

#include <iostream>
#include <sstream>

class first_actAction // this class has the action server definition
{
    /*these are the protected variables of the action class*/
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<first_pkg::first_actAction> as; /*creating a action server instance with our custom action message.
  node handle object must be created , as it would result in an error*/

    first_pkg::first_actFeedback feedback; // creating a feedback instance for sending feedback during operation
    first_pkg::first_actResult result;     // creating a result instance to send final result

    std::string action_name;
    int goal;
    int progress;

public:
    /*this is an action constructor , and an action server is created here bu taking an argument such as
    node handle , action_name , and excuteCB, where executeCB is the action callback*/

    first_actAction(std::string name) : as(nh_, name, boost::bind(&first_actAction::executeCB, this, _1), false),action_name(name)
    {
        as.registerPreemptCallback(boost::bind(&first_actAction::preemptCB, this));
        /*this line register a callback when the action is preempted.the preemptCb is the callback
        name executed when the is a preempt request from the action client*/
        as.start();
        /*start the action server*/
    }

    ~first_actAction()
    {
    }

    void preemptCB()
    {
        ROS_WARN("%s got preempted!", action_name.c_str());
        result.final_count = progress;
        as.setPreempted(result, "I got Preempted");
        /*
        sets the status of the active goal to preempted
        setPreempted() takes parameters:
        1] result to send back to client
        2] text message to send to client
        */
    }

    void executeCB(const first_pkg::first_actGoalConstPtr &goal)
    {
        if (!as.isActive() /*true if a goal is active*/ || as.isPreemptRequested() /*true if preempt is requested*/)
            return;

        ros::Rate rate(5);
        ROS_INFO("%s is processing the goal %d", action_name.c_str(), goal->count);
        for (progress = 1; progress <= goal->count; progress++) /*here we are starting the counter till it reaches the goal*/
        {
            if (!ros::ok()) /*if the node unexpectedly is shutdown*/
            {
                result.final_count = progress;
                as.setAborted(result, "I failed!"); // set the satatus of the active goal to be aborted
                ROS_INFO("%s Shutting down", action_name.c_str());
                break;
            }

            if (!as.isActive() || as.isPreemptRequested())
            {
                return;
            }
            /*this condition is tested here again in case a preempt request is sent or if the goal is inactive*/

            if (goal->count <= progress)
            {
                ROS_INFO("%s Succeeded at getting to goal %d", action_name.c_str(), goal->count);
                result.final_count = progress;
                as.setSucceeded(result);        //active goal is set to succeeded
            }
            /*when the goal is reached this code block is executed*/

            else
            {
                ROS_INFO("Setting to goal %d / %d", feedback.current_number, goal->count);
                feedback.current_number = progress;
                as.publishFeedback(feedback);
            }

            /*this code block is executed when there is still progress needed to reach the goal*/

            rate.sleep();
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "action_server");
    ROS_INFO("Starting First Action Server");
    first_actAction first_action_obj(ros::this_node::getName());
    ros::spin();
    return 0;
}