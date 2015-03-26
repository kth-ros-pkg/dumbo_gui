/*
 *  qnode.cpp
 *
 *  Created on: Aug 30, 2012
 *  Authors:   Francisco Viña 
 *            fevb <at> kth.se
 */

/* Copyright (c) 2012, Francisco Vina, CVAP, KTH
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the KTH nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
   ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL KTH BE LIABLE FOR ANY
   DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
   ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
// ROS message includes
#include <std_msgs/String.h>

#include <sstream>
#include <dumbo_dashboard/qnode.hpp>
#include <control_msgs/GripperCommand.h>


/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
  init_argc(argc),
  init_argv(argv)
{
  nh_ = ros::NodeHandle("~");
}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::on_init()
{
	// Add your ros communications here.

    // service clients for connecting/disconnecting to Dumbo
    soft_connect_dumbo_client = nh_.serviceClient<std_srvs::Empty>("/dumbo/soft_connect");
    connect_dumbo_client = nh_.serviceClient<std_srvs::Empty>("/dumbo/connect");
    disconnect_dumbo_client = nh_.serviceClient<std_srvs::Empty>("/dumbo/disconnect");

    // service clients for stopping/recovering Dumbo
    stop_dumbo_client = nh_.serviceClient<std_srvs::Empty>("/dumbo/stop");
    recover_dumbo_client = nh_.serviceClient<std_srvs::Empty>("/dumbo/recover");

    // service clients for connecting/disconnecting to left arm
    connect_left_arm_client = nh_.serviceClient<std_srvs::Empty>("/left_arm/connect");
    disconnect_left_arm_client = nh_.serviceClient<std_srvs::Empty>("/left_arm/disconnect");

    // service clients for connecting/disconnecting to right arm
    connect_right_arm_client = nh_.serviceClient<std_srvs::Empty>("/right_arm/connect");
    disconnect_right_arm_client = nh_.serviceClient<std_srvs::Empty>("/right_arm/disconnect");

    // service clients for connecting/disconnecting to PG70 parallel gripper
    connect_pg70_client = nh_.serviceClient<std_srvs::Empty>("/PG70_gripper/connect");
    disconnect_pg70_client = nh_.serviceClient<std_srvs::Empty>("/PG70_gripper/disconnect");
    recover_pg70_client = nh_.serviceClient<std_srvs::Empty>("/PG70_gripper/recover");

    // advertise PG70 parallel gripper position command
    pg70_pos_pub_ = nh_.advertise<control_msgs::GripperCommand>("/PG70_gripper/pos_command", 1);

    left_arm_status_sub_ = nh_.subscribe("/left_arm/connected", 1,  &QNode::leftArmStatusCallback, this);
    right_arm_status_sub_ = nh_.subscribe("/right_arm/connected", 1,  &QNode::rightArmStatusCallback, this);

    pg70_status_sub_ = nh_.subscribe("/PG70_gripper/connected", 1,  &QNode::pg70StatusCallback, this);

    left_arm_ft_sensor_status_sub_ = nh_.subscribe("/left_arm_ft_sensor/connected", 1,  &QNode::leftArmFTSensorStatusCallback, this);
    right_arm_ft_sensor_status_sub_= nh_.subscribe("/right_arm_ft_sensor/connected", 1,  &QNode::rightArmFTSensorStatusCallback, this);


	start();
	return true;
}

bool QNode::on_init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"test");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	// Add your ros communications here.
	start();
	return true;
}

void QNode::run() {
	ros::Rate loop_rate(10);
	int count = 0;

	while ( ros::ok() ) {
		ros::spinOnce();
		loop_rate.sleep();	
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	emit rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}



void QNode::sendGripperPos(double pos)
{
    control_msgs::GripperCommand pos_command_msg;
    pos_command_msg.position = pos/1000.0;

    pg70_pos_pub_.publish(pos_command_msg);
}

void QNode::leftArmStatusCallback(const std_msgs::Bool::ConstPtr &msg)
{
    if(msg->data)
    {
        emit leftArmConnected();
    }

    else
    {
        emit leftArmDisconnected();
    }
}

void QNode::rightArmStatusCallback(const std_msgs::Bool::ConstPtr &msg)
{
    if(msg->data)
    {
        emit rightArmConnected();
    }

    else
    {
        emit rightArmDisconnected();
    }
}

void QNode::pg70StatusCallback(const std_msgs::Bool::ConstPtr &msg)
{
    if(msg->data)
    {
        emit pg70Connected();
    }

    else
    {
        emit pg70Disconnected();
    }
}

void QNode::leftArmFTSensorStatusCallback(const std_msgs::Bool::ConstPtr &msg)
{
    if(msg->data)
    {
        emit leftArmFTSensorConnected();
    }

    else
    {
        emit leftArmFTSensorDisconnected();
    }
}

void QNode::rightArmFTSensorStatusCallback(const std_msgs::Bool::ConstPtr &msg)
{
    if(msg->data)
    {
        emit rightArmFTSensorConnected();
    }

    else
    {
        emit rightArmFTSensorDisconnected();
    }
}

