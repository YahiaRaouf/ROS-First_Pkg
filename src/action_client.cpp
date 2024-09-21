#include "ros/ros.h"
#include <iostream>
#include <actionlib/client/simple_action_client.h>      //is the action library used for implementing a simple action client
#include <actionlib/client/terminal_state.h>        //defines the possible goal states
#include "first_pkg/first_actAction.h"      //this includes the action messages from the first_act.action file

int main(int argc, char **argv)
{
    ros::init(argc, argv, "action_client");

    if (argc != 3)
    {
        ROS_INFO("%d", argc);
        ROS_WARN("Usage: first_act_client <goal> <time_to_preempt_in_sec>");
        return 1;
    }

    // create the action client
    // true causes the client to spin its own thread
    actionlib::SimpleActionClient<first_pkg::first_actAction> ac("action_server", true);

    ROS_INFO("Waiting for action server to start.");

    /* wait for the action server to connect to the action server.
       Often, it can take a second for the action server & client to negotiate a connection,
       thus, risking the first few goals to be dropped.
       This call lets the user wait until the network connection to the server is negotiated.*/
    ac.waitForServer(); // will wait for infinite time

    ROS_INFO("Action server started, sending goal.");

    // send a goal to the action
    first_pkg::first_actGoal goal;
    goal.count = atoi(argv[1]); //goal.count is equal to the first argument after running this node

    ROS_INFO("Sending Goal [%d] and Preempt time of [%d]", goal.count, atoi(argv[2]));
    ac.sendGoal(goal);      //sending the goal to the action server

    // wait for the action goal to finish before continuing.
    
    /* the waitForResult() blocks until the goal is finished.
       the parameter is the max tim
       before returning. A zero timeout is interpreted as infinite timeout
       returns true if the goal is finished. false if the goal didnt finish in 
       within the allocated time*/

    bool finished_before_timeout = ac.waitForResult(ros::Duration(atoi(argv[2])));
    ac.cancelGoal();        //cancels the goal.wad7a ya3ny

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac.getState();
        /*Get the state information for this goal.

          Possible States Are: PENDING, ACTIVE, RECALLED, REJECTED, PREEMPTED, ABORTED, SUCCEEDED, LOST.

          Returns
          The goal's state. Returns LOST if this SimpleActionClient isn't tracking a goal.*/
        ROS_INFO("Action finished: %s", state.toString().c_str());
        // Preempting the process
        ac.cancelGoal();
    }
    else
        ROS_INFO("Action did not finish before the time out.");

    // exit
    return 0;
}