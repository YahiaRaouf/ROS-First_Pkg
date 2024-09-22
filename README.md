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
Here i implemented a message or a struct of name custom_msgs that contain two variables , a string and an int.
they were used mainly in the Publisher and subscriber nodes.








