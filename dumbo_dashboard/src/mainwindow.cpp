/*
 *  mainwindow.cpp
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


#include <QtGui>
#include <iostream>
#include <dumbo_dashboard/mainwindow.hpp>
#include <dumbo_dashboard/qnode.hpp>

#ifndef PI
#define PI (3.141592653589793)
#endif 

MainWindow::MainWindow(int argc, char** argv, QWidget *parent) :
  QMainWindow(parent),
  qnode(argc,argv),
  ui(new Ui::MainWindow)
{
  ui->setupUi(this);
  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

  QObject::connect(&qnode, SIGNAL(leftArmConnected()), this, SLOT(display_leftarm_connected()));
  QObject::connect(&qnode, SIGNAL(leftArmDisconnected()), this, SLOT(display_leftarm_disconnected()));


  QObject::connect(&qnode, SIGNAL(rightArmConnected()), this, SLOT(display_rightarm_connected()));
  QObject::connect(&qnode, SIGNAL(rightArmDisconnected()), this, SLOT(display_rightarm_disconnected()));

  QObject::connect(&qnode, SIGNAL(pg70Connected()), this, SLOT(display_pg70_connected()));
  QObject::connect(&qnode, SIGNAL(pg70Disconnected()), this, SLOT(display_pg70_disconnected()));

  QObject::connect(&qnode, SIGNAL(leftArmFTSensorConnected()), this, SLOT(display_left_ft_connected()));
  QObject::connect(&qnode, SIGNAL(leftArmFTSensorDisconnected()), this, SLOT(display_left_ft_disconnected()));

  QObject::connect(&qnode, SIGNAL(rightArmFTSensorConnected()), this, SLOT(display_right_ft_connected()));

  QObject::connect(&qnode, SIGNAL(rightArmFTSensorDisconnected()), this, SLOT(display_right_ft_disconnected()));

  qnode.on_init();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_connect_button_clicked(bool check)
{
    std_srvs::Empty srv;

    if(!qnode.connect_dumbo_client.call(srv))
    {
        ROS_ERROR("Dumbo dashboard: error connecting to Dumbo.");
    }
}

void MainWindow::on_disconnect_button_clicked(bool check)
{
    std_srvs::Empty srv;

    if(!qnode.disconnect_dumbo_client.call(srv))
    {
        ROS_ERROR("Dumbo dashboard: error disconnecting from Dumbo.");
    }
}

void MainWindow::on_stop_button_clicked(bool check)
{
    std_srvs::Empty srv;

    if(!qnode.stop_dumbo_client.call(srv))
    {
        ROS_ERROR("Dumbo dashboard; error stopping Dumbo.");
    }
}

void MainWindow::on_recover_button_clicked(bool check)
{
    std_srvs::Empty srv;

    if(!qnode.recover_dumbo_client.call(srv))
    {
        ROS_ERROR("Dumbo dashboard: error recovering Dumbo.");
    }
}

void MainWindow::on_send_gripper_pos_button_clicked(bool check)
{
  qnode.sendGripperPos(ui->gripper_pos->text().toDouble());
}

void MainWindow::on_close_gripper_button_clicked(bool check)
{
  qnode.sendGripperPos(0.0);
}


void MainWindow::display_leftarm_connected()
{
  ui->label_LA->setText("Left arm connected");
}

void MainWindow::display_rightarm_connected()
{
  ui->label_RA->setText("Right arm connected");
}

void MainWindow::display_leftarm_disconnected()
{
  ui->label_LA->setText("Left arm disconnected");
}

void MainWindow::display_rightarm_disconnected()
{
  ui->label_RA->setText("Right arm disconnected");
}

void MainWindow::display_pg70_connected()
{
    ui->label_gripper->setText("PG70 gripper connected");
}

void MainWindow::display_pg70_disconnected()
{
    ui->label_gripper->setText("PG70 gripper disconnected");
}

void MainWindow::display_left_ft_connected()
{
    ui->label_l_ft->setText("Left arm F/T sensor connected");
}

void MainWindow::display_right_ft_connected()
{
    ui->label_r_ft->setText("Right arm F/T sensor connected");
}


void MainWindow::display_left_ft_disconnected()
{
    ui->label_l_ft->setText("Left arm F/T sensor disconnected");
}

void MainWindow::display_right_ft_disconnected()
{
    ui->label_r_ft->setText("Right arm F/T sensor disconnected");
}

