/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author: Michael X. Grey <mxgrey@gatech.edu>
 * Date: May 08, 2013
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 *
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */


#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>


#include "hubo_init.h"

namespace hubo_init_space
{

HuboInitPanel::HuboInitPanel(QWidget *parent)
    : rviz::Panel(parent)
{
    content = new HuboInitWidget();

    QHBoxLayout* panelLayout = new QHBoxLayout;
    panelLayout->addWidget(content);
    setLayout(panelLayout);
}


HuboInitWidget::HuboInitWidget(QWidget *parent)
    : QTabWidget(parent)
{
    initializeAch();


    // Set up the networking box
    achdConnect = new QPushButton;
    achdConnect->setText("Connect");
    achdConnect->setToolTip("Connect to motion computer");
    QHBoxLayout* networkLayout = new QHBoxLayout;
    networkLayout->addWidget(achdConnect);
    QGroupBox* networkBox = new QGroupBox;
    networkBox->setTitle("Ach Networking");
    networkBox->setLayout(networkLayout);
    ////////////


    // Set up the global command box
    homeAll = new QPushButton;
    homeAll->setText("Home All");
    homeAll->setToolTip("Execute homing command on all joints at once");
    connect(homeAll, SIGNAL(released()), this, SLOT(handleHomeAll()));

    homeBad->setText("Rehome");
    homeBad->setToolTip("Execute homing command only on joints that failed to home");
    connect(homeBad, SIGNAL(released()), this, SLOT(handleHomeBad()));

    initSensors->setText("Start Sensors");
    initSensors->setToolTip("Activate all the sensors and initialize them to zero");
    connect(initSensors, SIGNAL(released()), this, SLOT(handleInitSensors()));

    QLabel* refreshLabel;
    refreshLabel->setText("Refresh Rate:");
    refreshRate->setToolTip("Rate (Hz) at which state information is refreshed");
    refreshRate->setValue(10);
    refreshRate->setMaximum(200);
    refreshRate->setMinimum(1);
    QVBoxLayout* refreshLayout = new QVBoxLayout;
    refreshLayout->addWidget(refreshLabel);
    refreshLayout->addWidget(refreshRate);

    QHBoxLayout* globalCmdLayout = new QHBoxLayout;
    globalCmdLayout->addWidget(homeAll);
    globalCmdLayout->addWidget(homeBad);
    globalCmdLayout->addWidget(initSensors);
    globalCmdLayout->addLayout(refreshLayout);
    QGroupBox* globalCmdBox = new QGroupBox;
    globalCmdBox->setLayout(globalCmdLayout);
    /////////////


    // Set up the joint command box
    QHBoxLayout* radioLayout = new QHBoxLayout;
    radioCmdButtons = new QButtonGroup(this);

    home->setText("Home");
    home->setToolTip("Home a specific joint");
    radioCmdButtons->addButton(home);
    radioLayout->addWidget(home);

    reset->setText("Reset");
    reset->setToolTip("Reset encoder and clear error flags");
    radioCmdButtons->addButton(reset);
    radioLayout->addWidget(reset);

    ctrlOn->setText("Ctrl On");
    ctrlOn->setToolTip("Turn the motor control on for this joint's board");
    radioCmdButtons->addButton(ctrlOn);
    radioLayout->addWidget(ctrlOn);

    ctrlOff->setText("Ctrl Off");
    ctrlOff->setToolTip("Turn off the motor control for this joint's board");
    radioCmdButtons->addButton(ctrlOff);
    radioLayout->addWidget(ctrlOff);

    fetOn->setText("FET On");
    fetOn->setToolTip("Allow voltage through the motor");
    radioCmdButtons->addButton(fetOn);
    radioLayout->addWidget(fetOn);

    fetOff->setText("FET Off");
    fetOn->setToolTip("Disable voltage through the motor");
    radioCmdButtons->addButton(fetOff);
    radioLayout->addWidget(fetOff);

    zero->setText("Zero");
    zero->setToolTip("Zero out the current value of the encoder");
    radioCmdButtons->addButton(zero);
    radioLayout->addWidget(zero);

    jointCmdButtons.resize(HUBO_JOINT_COUNT);

    jointCmdGroup = new QButtonGroup(this);
    QHBoxLayout* jointCmdLayout = new QHBoxLayout;
    for(int i=0; i<HUBO_JOINT_COUNT; i++)
    {
        QPushButton* tempPushButton = new QPushButton;
        tempPushButton->setText(QString::fromLocal8Bit(jointNames[i]));
        tempPushButton->setToolTip("Normal");

        jointCmdGroup->addButton(tempPushButton, i);
        jointCmdLayout->addWidget(tempPushButton);

        jointCmdButtons[i] = tempPushButton;
    }

    QVBoxLayout* cmdLayout = new QVBoxLayout;
    cmdLayout->addLayout(radioLayout);
    cmdLayout->addLayout(jointCmdLayout);
    QGroupBox* jointCmdBox = new QGroupBox;
    jointCmdBox->setLayout(cmdLayout);
    ///////////////








    QVBoxLayout* masterCTLayout = new QVBoxLayout;
    masterCTLayout->addWidget(networkBox);
    masterCTLayout->addWidget(globalCmdBox);
    masterCTLayout->addWidget(jointCmdBox);

    commandTab->setLayout(masterCTLayout);



    // Next: jointStateTab, sensorCmdTab, and sensorStateTab

    addTab(commandTab, "Joint Command");
    addTab(jointStateTab, "Joint State");
    addTab(sensorCmdTab, "Sensor Command");
    addTab(sensorStateTab, "Sensor State");


}







} // End hubo_init_space
