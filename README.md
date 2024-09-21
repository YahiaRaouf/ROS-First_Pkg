# My First ROS Package

![ROS](https://img.shields.io/badge/ROS-Noetic-blue)

This repository contains my first ROS (Robot Operating System) package, showcasing fundamental ROS concepts including publishers, subscribers, services, action clients/servers, custom messages, and launch files.

## Table of Contents
- [Overview](#overview)
- [Features](#features)
- [Installation](#installation)
- [Package Structure](#package-structure)
- [Custom Messages](#custom-messages)
- [Usage](#usage)
  - [Publisher and Subscriber](#publisher-and-subscriber)
  - [Service Client and Server](#service-client-and-server)
  - [Action Client and Server](#action-client-and-server)
- [Launch Files](#launch-files)
- [Contributing](#contributing)
- [License](#license)

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
## clone the repo
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
│   └── MyAction.action       # Custom action file
├── msg/                      # Custom message definitions
│   └── MyCustomMsg.msg       # Example custom message
├── srv/                      # Custom service definitions
│   └── MyService.srv         # Example service definition
├── src/                      # Source code for nodes
│   ├── publisher_node.cpp    # Publisher node implementation
│   ├── subscriber_node.cpp   # Subscriber node implementation
│   ├── service_server.cpp    # Service server implementation
│   ├── service_client.cpp    # Service client implementation
│   ├── action_server.cpp     # Action server implementation
│   ├── action_client.cpp     # Action client implementation
├── launch/                   # Launch files
│   ├── pub_sub.launch        # Launch file for publisher and subscriber
│   ├── service.launch        # Launch file for service client and server
│   ├── action.launch         # Launch file for action client and server
├── CMakeLists.txt            # Configuration for CMake
├── package.xml               # Package metadata and dependencies
└── README.md                 # This file
```
