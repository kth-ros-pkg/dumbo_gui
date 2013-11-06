/*
 *  qnode.hpp
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
 * @file /include/test/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef test_QNODE_HPP_
#define test_QNODE_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <cob_srvs/Trigger.h>
#include <std_msgs/String.h>
#include <brics_actuator/JointPositions.h>
#include <dumbo_srvs/ClosePG70Gripper.h>
#include <dumbo_srvs/CalibrateFT.h>

/*****************************************************************************
 ** Class
 *****************************************************************************/

class QNode : public QThread {
  Q_OBJECT
  public:
  QNode(int argc, char** argv );
  virtual ~QNode();
  bool on_init();
  bool on_init(const std::string &master_url, const std::string &host_url);
  void run();

    
  void initArms();
  void disconnect_robot();
  void stopArms();
  void recoverArms();
  void sendGripperPos(double pos);
  void closeGripper(double target_vel, double current_limit);

  void connectFT();

signals:
  void rosShutdown();
  void leftArmConnected();
  void rightArmConnected();
  void leftArmDisconnected();
  void rightArmDisconnected();

private:
  int init_argc;
  char** init_argv;
  // ROS communication stuff
  ros::NodeHandle n;
  
  ros::ServiceClient left_arm_initsrv_client;
  ros::ServiceClient right_arm_initsrv_client;

  ros::ServiceClient left_arm_disconnectsrv_client;
  ros::ServiceClient right_arm_disconnectsrv_client;
  
  ros::ServiceClient pg70_initsrv_client;
  ros::ServiceClient sdh_initsrv_client;

  ros::ServiceClient pg70_disconnectsrv_client;
  ros::ServiceClient sdh_disconnectsrv_client;

  ros::ServiceClient left_arm_stopsrv_client;
  ros::ServiceClient right_arm_stopsrv_client;

  ros::ServiceClient left_arm_recoversrv_client;
  ros::ServiceClient right_arm_recoversrv_client;

  ros::ServiceClient pg70_recoversrv_client;
  ros::ServiceClient sdh_recoversrv_client;

  ros::ServiceClient left_arm_FT_connectsrv_client;
  ros::ServiceClient left_arm_FT_disconnectsrv_client;
  ros::ServiceClient left_arm_FT_calibsrv_client;

  ros::ServiceClient right_arm_FT_connectsrv_client;
  ros::ServiceClient right_arm_FT_disconnectsrv_client;
  ros::ServiceClient right_arm_FT_calibsrv_client;
  
  ros::Publisher pg70_pos_pub;
  ros::ServiceClient pg70_close_srv_client;

};


#endif /* test_QNODE_HPP_ */
