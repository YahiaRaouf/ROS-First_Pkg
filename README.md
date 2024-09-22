# My First ROS Package

![ROS](https://img.shields.io/badge/ROS-Noetic-blue)

This repository contains my first ROS (Robot Operating System) package, showcasing fundamental ROS concepts including publishers, subscribers, services, action clients/servers, custom messages, and launch files.

## Table of Contents
- [Overview](#overview)
- [Features](#features)
- [Installation](#installation)
- [Clone the repo](#clone-the-repo)
- [Build the package](#build-the-package)
- [Package structure](#package-structure)
- [Publisher and subscriber](#publisher-and-subscriber)
  - [Custom Message](#custom-message)
- [Service](#service)
- [Actionlib](#actionlib)

## Overview
This package serves as a learning project for ROS, implementing various communication mechanisms between nodes such as topics, services, and actions. It also demonstrates how to create custom messages and use ROS launch files to automate node execution.

### Implemented Nodes:
- **Publisher/Subscriber**: Implements a basic topic communication between nodes.
- **Service Client/Server**: Demonstrates synchronous communication using ROS services.
- **Action Client/Server**: Implements goal-oriented asynchronous communication using the action library.

## Features
- **Publisher and Subscriber Nodes**: Nodes that communicate using ROS topics.
- **Service Communication**: Synchronous service-based request-response communication.
- **Action Client and Server**: Asynchronous communication via goals and feedback using the action library.
- **Custom Messages**: Defines and uses custom message types in ROS.
- **Launch Files**: Launch files to start specific pairs of nodes.

## Installation

### Prerequisites
Ensure that ROS Noetic (or your ROS distribution) is installed on your system.

```bash
sudo apt update
sudo apt install ros-noetic-desktop-full
```
## Clone the repo
Navigate to your ROS workspace (usually ~/catkin_ws/src) and clone the repository:
```bash
cd ~/catkin_ws/src
git clone https://github.com/username/my_first_ros_package.git
```
## build the package
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```
## Package structure
```bash
my_first_ros_package/
├── action/                   # Action definitions
│   └── first_act.action      # Custom action file
├── msg/                      # Custom message definitions
│   └── custom_msg.msg        # Example custom message
├── srv/                      # Custom service definitions
│   └── first_srv.srv         # Example service definition
├── src/                      # Source code for nodes
│   ├── pub.cpp               # Publisher node implementation
│   ├── sub.cpp               # Subscriber node implementation
│   ├── server.cpp            # Service server implementation
│   ├── client.cpp            # Service client implementation
│   ├── action_server.cpp     # Action server implementation
│   ├── action_client.cpp     # Action client implementation
├── launch/                   # Launch files
│   ├── pubToSub.launch       # Launch file for publisher and subscriber
│   ├── clientToServer.launch # Launch file for service client and server
├── CMakeLists.txt            # Configuration for CMake
├── package.xml               # Package metadata and dependencies
└── README.md                 # This file
```
## Publisher and subscriber
Publsihers and subscriber nodes are the most basic program you can implement as a ROS beginner , basically the both  nodes talk to each other using a certian type of messsage , commmunicating through Topics.

![Topic-SinglePublisherandSingleSubscriber](https://github.com/user-attachments/assets/6fedf89a-48be-42cc-a9e8-a76d5073afb3)

to launch the pub and sub nodes in the first_pkg ,  cd to the workspace
```bash
cd ~/catkin_ws    #your workspace can hold any name , but catkin_ws is the usual
```
and then use the command 
```bash
roslaunch first_pkg pubToSub.launch
```
you will see a string and integer message sent from the publisher node to the subscriber node using the custom_msgs declared in the msg folder in the package

### Custom message
messages are the way of communication between nodes , their unique language.Both nodes should use the same messages with each other , and you can create your own messages
by creating a .msg file in the msg folder.

the file should look like somtheing similar to this

```bash
std_msgs/String name
std_msgs/Int32 number
```
Here i implemented a message or a struct of name custom_msgs that contain two variables , a string and an integer.
they were used mainly in the Publisher and subscriber nodes.

## Service
a Service is another way of communication between nodes , unlike the publisher and subscriber where the communication between the nodes is continious , the Service's client starts to request a service from the service's server. only then ,  the server start doing what its programed to do.

![Service-SingleServiceClient](https://github.com/user-attachments/assets/7336bbe2-429f-484e-a954-03831953b978)

to launch the server/client launch file 
```bash
cd ~/catkin_ws
roslaunch first_pkg clientToServer
```
a service needs a type of message to be requested and responded to , which is created in a .srv file in the srv folder inside the package.
for example , first_srv.srv file:
```bash
string in
---
string out
```
as easy as that :)
services are mainly used when you want a specific action done at a specific time.
like when turning a robot sensor off , or when you want to move an arm to a specific location when an action is done.
and so on.

## Actionlib
The ROS Action Library is used for asynchronous communication between nodes, allowing long-running tasks to be executed without blocking the client. Actions are an extension of ROS services but provide more flexibility, especially for tasks that require feedback, the ability to cancel, or take an indefinite amount of time to complete.

![Action-SingleActionClient](https://github.com/user-attachments/assets/7895183a-af8b-4359-94c7-2fa259920db5)

to declare an action you have to create an aciton file in the action folder like first_act.action.
which is written like this: 

```bash
#goal definition
int32 count
---
#result definition
int32 final_count
---
#feedback
int32 current_number
```
here we defined a data type for the goal , the result and the feedback variable of the action.
the implementation in this package mainly cancles teh client request if the server didnt achieve the goal in a certian time and sends another request in hope it will be achieved this time.

to start the nodes. first start the server , then , the client.
```bash
cd ~/catkin_ws
rosrun first_pkg action_server
rosrun first_pkg action_client [arg1] [arg2]
```
arg1 is the goal to be reached by the server.
arg2 is the time in which the operation is canceled after.
