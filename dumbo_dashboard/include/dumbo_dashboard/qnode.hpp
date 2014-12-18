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
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
/*****************************************************************************
 ** Class
 *****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
    // service clients for connecting/disconnecting stopping/recovering HW
    ros::ServiceClient soft_connect_dumbo_client;
    ros::ServiceClient connect_dumbo_client;
    ros::ServiceClient disconnect_dumbo_client;

    ros::ServiceClient stop_dumbo_client;
    ros::ServiceClient recover_dumbo_client;

    ros::ServiceClient connect_left_arm_client;
    ros::ServiceClient disconnect_left_arm_client;

    ros::ServiceClient connect_right_arm_client;
    ros::ServiceClient disconnect_right_arm_client;

    ros::ServiceClient connect_pg70_client;
    ros::ServiceClient disconnect_pg70_client;
    ros::ServiceClient recover_pg70_client;


    QNode(int argc, char** argv );
    virtual ~QNode();
    bool on_init();
    bool on_init(const std::string &master_url, const std::string &host_url);
    void run();

    

    void sendGripperPos(double pos);

    void leftArmStatusCallback(const std_msgs::Bool::ConstPtr &msg);
    void rightArmStatusCallback(const std_msgs::Bool::ConstPtr &msg);

    void pg70StatusCallback(const std_msgs::Bool::ConstPtr &msg);

    void leftArmFTSensorStatusCallback(const std_msgs::Bool::ConstPtr &msg);
    void rightArmFTSensorStatusCallback(const std_msgs::Bool::ConstPtr &msg);

signals:
    void rosShutdown();

    void leftArmConnected();
    void leftArmDisconnected();

    void rightArmConnected();
    void rightArmDisconnected();

    void pg70Connected();
    void pg70Disconnected();

    void leftArmFTSensorConnected();
    void leftArmFTSensorDisconnected();

    void rightArmFTSensorConnected();
    void rightArmFTSensorDisconnected();

private:
    int init_argc;
    char** init_argv;
    // ROS communication stuff
    ros::NodeHandle nh_;

    ros::Publisher pg70_pos_pub_;

    // subscribers for HW status
    ros::Subscriber left_arm_status_sub_;
    ros::Subscriber right_arm_status_sub_;
    ros::Subscriber pg70_status_sub_;
    ros::Subscriber left_arm_ft_sensor_status_sub_;
    ros::Subscriber right_arm_ft_sensor_status_sub_;

};


#endif /* test_QNODE_HPP_ */
