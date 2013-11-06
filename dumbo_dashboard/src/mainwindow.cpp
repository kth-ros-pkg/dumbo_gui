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
#include "../include/dumbo_dashboard/mainwindow.hpp"
#include "../include/dumbo_dashboard/qnode.hpp"

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
  QObject::connect(&qnode, SIGNAL(rightArmConnected()), this, SLOT(display_rightarm_connected()));
  QObject::connect(&qnode, SIGNAL(leftArmDisconnected()), this, SLOT(display_leftarm_disconnected()));
  QObject::connect(&qnode, SIGNAL(rightArmDisconnected()), this, SLOT(display_rightarm_disconnected()));
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_connect_button_clicked(bool check)
{
  qnode.on_init();
  qnode.initArms();
  qnode.connectFT();
}

void MainWindow::on_disconnect_button_clicked(bool check)
{
    qnode.disconnect_robot();
}

void MainWindow::on_stop_button_clicked(bool check)
{
  qnode.stopArms();
}

void MainWindow::on_recover_button_clicked(bool check)
{
  qnode.recoverArms();
}

void MainWindow::on_send_gripper_pos_button_clicked(bool check)
{
  qnode.sendGripperPos(ui->gripper_pos->text().toDouble());
}

void MainWindow::on_close_gripper_button_clicked(bool check)
{
  qnode.closeGripper(ui->gripper_target_vel->text().toDouble()/1000.0,
		     ui->gripper_current_limit->text().toDouble());
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

