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


/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
  init_argc(argc),
  init_argv(argv)
{
  n = ros::NodeHandle("~"); 
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

	// subscribe to arm initialization services
	left_arm_initsrv_client = n.serviceClient<cob_srvs::Trigger>("/left_arm_controller/init");
	right_arm_initsrv_client = n.serviceClient<cob_srvs::Trigger>("/right_arm_controller/init");

    // subscribe to disconnect services
    left_arm_disconnectsrv_client = n.serviceClient<cob_srvs::Trigger>("/left_arm_controller/disconnect");
    right_arm_disconnectsrv_client = n.serviceClient<cob_srvs::Trigger>("/right_arm_controller/disconnect");

	// subscribe to PG70 parallel gripper initialization services
	pg70_initsrv_client = n.serviceClient<cob_srvs::Trigger>("/PG70_controller/init");
	sdh_initsrv_client = n.serviceClient<cob_srvs::Trigger>("/sdh_controller/init");

    // subscribe to PG70 parallel gripper disconnect services
    pg70_disconnectsrv_client = n.serviceClient<cob_srvs::Trigger>("/PG70_controller/disconnect");
    sdh_disconnectsrv_client = n.serviceClient<cob_srvs::Trigger>("/sdh_controller/shutdown");

	// subscribe to arm stop services
	left_arm_stopsrv_client = n.serviceClient<cob_srvs::Trigger>("/left_arm_controller/stop_arm");
	right_arm_stopsrv_client = n.serviceClient<cob_srvs::Trigger>("/right_arm_controller/stop_arm");

	// subscribe to arm recover services
	left_arm_recoversrv_client = n.serviceClient<cob_srvs::Trigger>("/left_arm_controller/recover");
	right_arm_recoversrv_client = n.serviceClient<cob_srvs::Trigger>("/right_arm_controller/recover");

	// subscribe to gripper recover services
	pg70_recoversrv_client = n.serviceClient<cob_srvs::Trigger>("/PG70_controller/recover");
	sdh_recoversrv_client = n.serviceClient<cob_srvs::Trigger>("/sdh_controller/recover");

	// subscribe to gripper pos
	pg70_pos_pub = n.advertise<brics_actuator::JointPositions>("/PG70_controller/command_pos", 1);

	// subscribe to FT sensor init service
    left_arm_ft_connectsrv_client = n.serviceClient<cob_srvs::Trigger>("/left_arm_ft_sensor/connect");
    left_arm_ft_disconnectsrv_client = n.serviceClient<cob_srvs::Trigger>("/left_arm_ft_sensor/disconnect");
    left_arm_ft_calibsrv_client = n.serviceClient<dumbo_srvs::CalibrateFT>("/left_arm_ft_sensor/calibrate");
    right_arm_ft_connectsrv_client = n.serviceClient<cob_srvs::Trigger>("/right_arm_ft_sensor/connect");
    right_arm_ft_disconnectsrv_client = n.serviceClient<cob_srvs::Trigger>("/right_arm_ft_sensor/disconnect");
    right_arm_ft_calibsrv_client = n.serviceClient<dumbo_srvs::CalibrateFT>("/right_arm_ft_sensor/calibrate");


	// subscribe to left gripper close service
	pg70_close_srv_client = n.serviceClient<dumbo_srvs::ClosePG70Gripper>("/PG70_controller/close_gripper");

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

void QNode::initArms()
{
  cob_srvs::Trigger left_arm_init_srv;

  while(!left_arm_initsrv_client.exists())
  {
	  ROS_INFO("Waiting for left arm init service");
	  ros::Duration(1).sleep();
  }

  if(left_arm_initsrv_client.call(left_arm_init_srv))
    {
      if(left_arm_init_srv.response.success.data) 
	{
	  ROS_INFO("Successfully initialized left arm...");
	  emit leftArmConnected();
	}
      else
	{
	  ROS_ERROR("Error initializing left arm");
	}
    }

  else
    {
      ROS_ERROR("Failed to call left arm init service");
    }


  // open the PG70 gripper if the left arm connected correctly
  if(left_arm_init_srv.response.success.data)
  {
	  cob_srvs::Trigger pg70_init_srv;

//	  while(!pg70_initsrv_client.exists())
//	  {
//		  ROS_INFO("Waiting for PG70 init service");
//		  ros::Duration(1).sleep();
//	  }

	  if(pg70_initsrv_client.call(pg70_init_srv))
	  {
		  if(pg70_init_srv.response.success.data)
		  {
			  ROS_INFO("Successfully initialized PG70 gripper...");
			  //		  emit rightArmConnected(); todo fix for pg70
		  }
		  else
		  {
			  ROS_ERROR("Error initializing PG70 gripper");
		  }
	  }

	  else
	  {
		  ROS_ERROR("Failed to call PG70 gripper init service");
	  }
  }


  cob_srvs::Trigger right_arm_init_srv;

  while(!right_arm_initsrv_client.exists())
  {
	  ROS_INFO("Waiting for right arm init service");
	  ros::Duration(1).sleep();
  }

  if(right_arm_initsrv_client.call(right_arm_init_srv))
  {
	  if(right_arm_init_srv.response.success.data)
	  {
		  ROS_INFO("Successfully initialized right arm...");
		  emit rightArmConnected();
	  }
	  else
	  {
		  ROS_ERROR("Error initializing right arm");
	  }
  }

  else
  {
	  ROS_ERROR("Failed to call right arm init service");
  }


  if(right_arm_init_srv.response.success.data)
  {
	  cob_srvs::Trigger sdh_init_srv;

//	  while(!sdh_initsrv_client.exists())
//	  {
//		  ROS_INFO("Waiting for SDH init service");
//		  ros::Duration(1).sleep();
//	  }

	  if(sdh_initsrv_client.call(sdh_init_srv))
	  {
		  if(sdh_init_srv.response.success.data)
		  {
			  ROS_INFO("Successfully initialized SDH gripper...");
			  //		  emit rightArmConnected(); todo fix for sdh
		  }
		  else
		  {
			  ROS_ERROR("Error initializing SDH gripper");
		  }
	  }

	  else
	  {
		  ROS_ERROR("Failed to call SDH gripper init service");
	  }
  }


}

void QNode::disconnect_robot()
{
    cob_srvs::Trigger disconnect_srv;

    pg70_disconnectsrv_client.call(disconnect_srv);
    left_arm_disconnectsrv_client.call(disconnect_srv);
    sdh_disconnectsrv_client.call(disconnect_srv);
    right_arm_disconnectsrv_client.call(disconnect_srv);

    left_arm_ft_disconnectsrv_client.call(disconnect_srv);
    right_arm_ft_disconnectsrv_client.call(disconnect_srv);
    emit leftArmDisconnected();
    emit rightArmDisconnected();
    emit left_ft_disconnected();
    emit right_ft_disconnected();
}


void QNode::stopArms()
{
  cob_srvs::Trigger left_arm_stop_srv;
  if(left_arm_stopsrv_client.call(left_arm_stop_srv))
    {
      if(left_arm_stop_srv.response.success.data) 
	{
	  ROS_INFO("Successfully stopped left arm...");
	}
      else
	{
	  ROS_ERROR("Error stopping left arm");
	}
    }

  else
    {
      ROS_ERROR("Failed to call left arm stop service");
    }


  cob_srvs::Trigger right_arm_stop_srv;
  if(right_arm_stopsrv_client.call(right_arm_stop_srv))
    {
      if(right_arm_stop_srv.response.success.data) 
	{
	  ROS_INFO("Successfully stopped right arm...");
	}
      else
	{
	  ROS_ERROR("Error stopping right arm");
	}
    }

  else
    {
      ROS_ERROR("Failed to call right arm stop service");
    }
}



void QNode::recoverArms()
{
  cob_srvs::Trigger left_arm_recover_srv;
  if(left_arm_recoversrv_client.call(left_arm_recover_srv))
    {
      if(left_arm_recover_srv.response.success.data==true)
	{
	  ROS_INFO("Successfully recovered left arm...");
	}
      else
	{
	  ROS_ERROR("Error recovering left arm");
	}
    }

  else
    {
      ROS_ERROR("Failed to call left arm recover service");
    }

  cob_srvs::Trigger pg70_recover_srv;
  if(pg70_recoversrv_client.call(pg70_recover_srv))
  {
	  if(pg70_recover_srv.response.success.data==true)
	  {
		  ROS_INFO("Successfully recovered PG70 parallel gripper...");
	  }
	  else
	  {
		  ROS_ERROR("Error recovering PG70 parallel gripper");
	  }
  }

  else
  {
	  ROS_ERROR("Failed to call PG70 parallel gripper recover service");
  }


  cob_srvs::Trigger right_arm_recover_srv;
  if(right_arm_recoversrv_client.call(right_arm_recover_srv))
    {
      if(right_arm_recover_srv.response.success.data==true)
	{
	  ROS_INFO("Successfully recovered right arm...");
	}
      else
	{
	  ROS_ERROR("Error recovering right arm");
	}
    }

  else
    {
      ROS_ERROR("Failed to call right arm recover service");
    }
}

void QNode::sendGripperPos(double pos)
{
  // *** hard coded...
  brics_actuator::JointPositions left_gripper_pos_command;
  left_gripper_pos_command.positions.resize(1);

  left_gripper_pos_command.positions[0].timeStamp = ros::Time::now();
  left_gripper_pos_command.positions[0].joint_uri = "left_arm_top_finger_joint";
  left_gripper_pos_command.positions[0].unit = "mm";
  left_gripper_pos_command.positions[0].value = pos;

  pg70_pos_pub.publish(left_gripper_pos_command);

}

void QNode::closeGripper(double target_vel, double current_limit)
{
	dumbo_srvs::ClosePG70Gripper gripper_close_srv;
	gripper_close_srv.request.target_vel = target_vel;
	gripper_close_srv.request.current_limit = current_limit;

	if(pg70_close_srv_client.call(gripper_close_srv))
	{
		if(gripper_close_srv.response.success)
		{
			ROS_INFO("Closing PG70 gripper");
		}

		else
		{
			ROS_ERROR("Couldn't close PG70 gripper.");
		}
	}

	else
	{
		ROS_ERROR("Couldn't close PG70 gripper.");
	}

}


void QNode::connectFT()
{

    cob_srvs::Trigger left_arm_ft_connect_srv;
    while(!left_arm_ft_connectsrv_client.exists())
	{
        ROS_INFO("Waiting for left arm F/T connect service.");
		ros::Duration(1).sleep();
	}
    if(left_arm_ft_connectsrv_client.call(left_arm_ft_connect_srv))
	{
        if(left_arm_ft_connect_srv.response.success.data)
		{
            ROS_INFO("Successfully connected to left arm F/T sensor...");
            emit left_ft_connected();
		}
		else
		{
            ROS_ERROR("Error connecting to left arm F/T sensor");
		}
	}

	else
	{
        ROS_ERROR("Failed to call left arm F/T sensor connect service");
	}

    cob_srvs::Trigger right_arm_ft_connect_srv;
    while(!right_arm_ft_connectsrv_client.exists())
	{
        ROS_INFO("Waiting for right arm F/T connect service.");
		ros::Duration(1).sleep();
	}
    if(right_arm_ft_connectsrv_client.call(right_arm_ft_connect_srv))
	{
        if(right_arm_ft_connect_srv.response.success.data)
		{
            ROS_INFO("Successfully connected to right arm F/T sensor...");
            emit right_ft_connected();
		}
		else
		{
            ROS_ERROR("Error connecting to right arm F/T sensor");
		}
	}

	else
	{
        ROS_ERROR("Failed to call right arm F/T sensor connect service");
	}

}
