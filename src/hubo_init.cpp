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
#include "FlowLayout.h"

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
    //TODO: initializeAch();

    groupStyleSheet = "QGroupBox {"
                      "border: 1px solid gray;"
                      "border-radius: 9px;"
                      "margin-top: 0.5em;"
                      "}"
                      "QGroupBox::title {"
                      "subcontrol-origin: margin;"
                      "left: 10px;"
                      "padding: 0 3px 0 3px;"
                      "}";


    // Set up the networking box
    achdConnect = new QPushButton;
    achdConnect->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    achdConnect->setText("Connect");
    achdConnect->setToolTip("Connect to motion computer");
    QHBoxLayout* networkLayout = new QHBoxLayout;
    networkLayout->addWidget(achdConnect);
    QGroupBox* networkBox = new QGroupBox;
    networkBox->setStyleSheet(groupStyleSheet);
    networkBox->setTitle("Ach Networking");
    networkBox->setLayout(networkLayout);
    ////////////


    // Set up the global command box
    homeAll = new QPushButton;
    homeAll->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    homeAll->setText("Home All");
    homeAll->setToolTip("Execute homing command on all joints at once");
    connect(homeAll, SIGNAL(released()), this, SLOT(handleHomeAll()));

    homeBad = new QPushButton;
    homeBad->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    homeBad->setText("Rehome");
    homeBad->setToolTip("Execute homing command only on joints that failed to home");
    connect(homeBad, SIGNAL(released()), this, SLOT(handleHomeBad()));

    initSensors = new QPushButton;
    initSensors->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    initSensors->setText("Start Sensors");
    initSensors->setToolTip("Activate all the sensors and initialize them to zero");
    connect(initSensors, SIGNAL(released()), this, SLOT(handleInitSensors()));

    QLabel* refreshLabel = new QLabel;
    refreshLabel->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    refreshLabel->setText("Refresh Rate:");
    refreshLabel->setToolTip("Rate (Hz) at which state information is refreshed");
    refreshRate = new QSpinBox;
    refreshRate->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
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
    globalCmdBox->setStyleSheet(groupStyleSheet);
    globalCmdBox->setTitle("Initialization Commands");
    globalCmdBox->setLayout(globalCmdLayout);
    /////////////


    // Set up the joint command box
    QHBoxLayout* radioLayout = new QHBoxLayout;
    radioCmdButtons = new QButtonGroup(this);

    radioCmdButtons->setExclusive(true);

    home = new QRadioButton;
    home->setText("Home");
    home->setToolTip("Home a specific joint");
    home->setChecked(true);
    radioCmdButtons->addButton(home);
    radioLayout->addWidget(home);

    reset = new QRadioButton;
    reset->setText("Reset");
    reset->setToolTip("Reset encoder and clear error flags");
    radioCmdButtons->addButton(reset);
    radioLayout->addWidget(reset);

    ctrlOn = new QRadioButton;
    ctrlOn->setText("Ctrl On");
    ctrlOn->setToolTip("Turn the motor control on for this joint's board");
    radioCmdButtons->addButton(ctrlOn);
    radioLayout->addWidget(ctrlOn);

    ctrlOff = new QRadioButton;
    ctrlOff->setText("Ctrl Off");
    ctrlOff->setToolTip("Turn off the motor control for this joint's board");
    radioCmdButtons->addButton(ctrlOff);
    radioLayout->addWidget(ctrlOff);

    fetOn = new QRadioButton;
    fetOn->setText("FET On");
    fetOn->setToolTip("Allow voltage through the motor");
    radioCmdButtons->addButton(fetOn);
    radioLayout->addWidget(fetOn);

    fetOff = new QRadioButton;
    fetOff->setText("FET Off");
    fetOff->setToolTip("Disable voltage through the motor");
    radioCmdButtons->addButton(fetOff);
    radioLayout->addWidget(fetOff);

    zero = new QRadioButton;
    zero->setText("Zero");
    zero->setToolTip("Zero out the current value of the encoder");
    radioCmdButtons->addButton(zero);
    radioLayout->addWidget(zero);

    jointCmdButtons.resize(HUBO_JOINT_COUNT);
    jointCmdGroup = new QButtonGroup(this);
    connect(jointCmdGroup, SIGNAL(buttonClicked(int)), this, SLOT(handleJointCmdButton(int)));
    FlowLayout* jointCmdLayout = new FlowLayout;
    for(int i=0; i<HUBO_JOINT_COUNT; i++)
    {
        QPushButton* tempPushButton = new QPushButton;
        tempPushButton->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
        tempPushButton->setText(QString::fromLocal8Bit(jointNames[i]));
        tempPushButton->setToolTip("Normal");

        jointCmdGroup->addButton(tempPushButton, i);
        jointCmdLayout->addWidget(tempPushButton);

        jointCmdButtons[i] = tempPushButton;
    }

    QVBoxLayout* cmdLayout = new QVBoxLayout;
    cmdLayout->addLayout(radioLayout);
    cmdLayout->addSpacing(15);
    cmdLayout->addLayout(jointCmdLayout);
    QGroupBox* jointCmdBox = new QGroupBox;
    jointCmdBox->setStyleSheet(groupStyleSheet);
    jointCmdBox->setTitle("Joint Commands");
    jointCmdBox->setLayout(cmdLayout);
    ///////////////


    QVBoxLayout* masterCTLayout = new QVBoxLayout;
    masterCTLayout->addWidget(networkBox);
    masterCTLayout->addWidget(globalCmdBox);
    masterCTLayout->addWidget(jointCmdBox);

    commandTab = new QWidget;
    commandTab->setLayout(masterCTLayout);

    std::cerr << "Command Tab complete" << std::endl;
    

    stateFlags = new QLineEdit;
    stateFlags->setReadOnly(true);

    jointStateButtons.resize(HUBO_JOINT_COUNT);
    jointStateGroup = new QButtonGroup(this);
    connect(jointStateGroup, SIGNAL(buttonClicked(int)), this, SLOT(handleJointStateButton(int)));
    FlowLayout* jointStateLayout = new FlowLayout;
    for(int i=0; i<HUBO_JOINT_COUNT; i++)
    {
        QPushButton* tempPushButton = new QPushButton;
        tempPushButton->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
        tempPushButton->setText(QString::fromLocal8Bit(jointNames[i]));
        tempPushButton->setToolTip("Normal");
        
        jointStateGroup->addButton(tempPushButton, i);
        jointStateLayout->addWidget(tempPushButton);
        
        jointStateButtons[i] = tempPushButton;
    }
    
    QVBoxLayout* masterJSTLayout = new QVBoxLayout;
    masterJSTLayout->addWidget(stateFlags);
    masterJSTLayout->addLayout(jointStateLayout);

    jointStateTab = new QWidget;
    jointStateTab->setLayout(masterJSTLayout);
    
    std::cerr << "Joint State Tab complete" << std::endl;

    radioSensorButtons = new QButtonGroup;
    radioSensorButtons->setExclusive(true);

    QHBoxLayout* radioSensorLayout = new QHBoxLayout;
    nullSensor = new QRadioButton;
    nullSensor->setText("Null Sensor");
    nullSensor->setToolTip("Set sensor values to zero. Must run this before data can be received.");
    radioSensorLayout->addWidget(nullSensor);
    nullSensor->setChecked(true);
    // Note: Leaving out initSensor because I think it's a dangerous feature

    QHBoxLayout* sensorLayout1 = new QHBoxLayout;
    lhFTButton = new QPushButton;
    lhFTButton->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    lhFTButton->setText("Left Hand FT");
    lhFTButton->setToolTip("Left hand force torque sensor");
    sensorLayout1->addWidget(lhFTButton);
    connect( lhFTButton, SIGNAL(clicked()), this, SLOT(handleLHFT()) );
    rhFTButton = new QPushButton;
    rhFTButton->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    rhFTButton->setText("Right Hand FT");
    rhFTButton->setToolTip("Right hand force torque sensor");
    sensorLayout1->addWidget(rhFTButton);
    connect( rhFTButton, SIGNAL(clicked()), this, SLOT(handleRHFT()) );

    imuButton = new QPushButton;
    imuButton->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    imuButton->setText("IMU");
    imuButton->setToolTip("Inertial Measurement Unit (waist)");
    connect( imuButton, SIGNAL(clicked()), this, SLOT(handleIMU()) );

    QHBoxLayout* sensorLayout3 = new QHBoxLayout;
    lfFTButton = new QPushButton;
    lfFTButton->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    lfFTButton->setText("Left Foot FT");
    lfFTButton->setToolTip("Left foot force torque sensor");
    sensorLayout3->addWidget(lfFTButton);
    connect( lfFTButton, SIGNAL(clicked()), this, SLOT(handleLFFT()) );
    rfFTButton = new QPushButton;
    rfFTButton->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    rfFTButton->setText("Right Foot FT");
    rfFTButton->setToolTip("Right foot force torque sensor");
    sensorLayout3->addWidget(rfFTButton);
    connect( rfFTButton, SIGNAL(clicked()), this, SLOT(handleRFFT()) );


    QVBoxLayout* masterSCTLayout = new QVBoxLayout;
    masterSCTLayout->setAlignment(Qt::AlignHCenter | Qt::AlignTop);
    masterSCTLayout->addLayout(radioSensorLayout);
    masterSCTLayout->addLayout(sensorLayout1);
    masterSCTLayout->addWidget(imuButton, 0, Qt::AlignCenter);
    masterSCTLayout->addLayout(sensorLayout3);

    sensorCmdTab = new QWidget;
    sensorCmdTab->setLayout(masterSCTLayout);


    std::cerr << "Sensor Command Tab complete" << std::endl;

    QVBoxLayout* sensorStateLayout = new QVBoxLayout;


    QGroupBox* ftBox = new QGroupBox;
    ftBox->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
    ftBox->setAlignment(Qt::AlignHCenter | Qt::AlignTop);
    ftBox->setTitle("Force-Torque Readings");
    ftBox->setStyleSheet(groupStyleSheet);

    QGridLayout* ftStateLayout = new QGridLayout;
    ftStateLayout->setAlignment(Qt::AlignHCenter | Qt::AlignTop);

    QLabel* lhftLab = new QLabel;
    lhftLab->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    lhftLab->setText("Left Hand");
    ftStateLayout->addWidget(lhftLab, 0, 1, 1, 1, Qt::AlignHCenter | Qt::AlignVCenter);

    QLabel* rhftLab = new QLabel;
    rhftLab->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    rhftLab->setText("Right Hand");
    ftStateLayout->addWidget(rhftLab, 0, 2, 1, 1, Qt::AlignHCenter | Qt::AlignVCenter);

    QLabel* lfftLab = new QLabel;
    lfftLab->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    lfftLab->setText("Left Foot");
    ftStateLayout->addWidget(lfftLab, 0, 3, 1, 1, Qt::AlignHCenter | Qt::AlignVCenter);

    QLabel* rfftLab = new QLabel;
    rfftLab->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    rfftLab->setText("Right Foot");
    ftStateLayout->addWidget(rfftLab, 0, 4, 1, 1, Qt::AlignHCenter | Qt::AlignVCenter);

    QLabel* mxLab = new QLabel;
    mxLab->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    mxLab->setText("X Moment");
    mxLab->setToolTip("Moment about the X-Axis (N-m)");
    ftStateLayout->addWidget(mxLab, 1, 0, 1, 1, Qt::AlignHCenter | Qt::AlignVCenter);

    QLabel* myLab = new QLabel;
    myLab->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    myLab->setText("Y Moment");
    myLab->setToolTip("Moment about the Y-Axis (N-m)");
    ftStateLayout->addWidget(myLab, 2, 0, 1, 1, Qt::AlignHCenter | Qt::AlignVCenter);

    QLabel* fzLab = new QLabel;
    myLab->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    fzLab->setText("Z Force");
    fzLab->setToolTip("Force along the Z-Axis (N)");
    ftStateLayout->addWidget(fzLab, 3, 0, 1, 1, Qt::AlignHCenter | Qt::AlignVCenter);

    ft_mx.resize(4);
    ft_my.resize(4);
    ft_fz.resize(4);
    for(int i=0; i<4; i++)
    {
        QLineEdit* tempMxEdit = new QLineEdit;
        tempMxEdit->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
        tempMxEdit->setReadOnly(true);
        ftStateLayout->addWidget(tempMxEdit, 1, i+1, 1, 1);
        ft_mx[i] = tempMxEdit;

        QLineEdit* tempMyEdit = new QLineEdit;
        tempMyEdit->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
        tempMyEdit->setReadOnly(true);
        ftStateLayout->addWidget(tempMyEdit, 2, i+1, 1, 1);
        ft_my[i] = tempMyEdit;

        QLineEdit* tempFzEdit = new QLineEdit;
        tempFzEdit->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
        tempFzEdit->setReadOnly(true);
        ftStateLayout->addWidget(tempFzEdit, 3, i+1, 1, 1);
        ft_fz[i] = tempFzEdit;
    }

    ftBox->setLayout(ftStateLayout);
    sensorStateLayout->addWidget(ftBox, 0, Qt::AlignHCenter | Qt::AlignTop);


    imuBox = new QGroupBox;
    imuBox->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    imuBox->setAlignment(Qt::AlignHCenter | Qt::AlignTop);
    imuBox->setStyleSheet(groupStyleSheet);
    imuBox->setTitle("IMU Sensor Readings");

    QGridLayout* imuStateLayout = new QGridLayout;
    imuStateLayout->setAlignment(Qt::AlignHCenter | Qt::AlignTop);

    QLabel* tiltx = new QLabel;
    tiltx->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    tiltx->setText("Tilt X");
    tiltx->setToolTip("Angle of tilt about the X-Axis (deg)");
    imuStateLayout->addWidget(tiltx, 0, 0, 1, 1, Qt::AlignLeft | Qt::AlignBottom );

    a_x = new QLineEdit;
    a_x->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    a_x->setReadOnly(true);
    imuStateLayout->addWidget(a_x, 1, 0, 1, 1, Qt::AlignCenter);

    QLabel* tilty = new QLabel;
    tilty->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    tilty->setText("Tilt Y");
    tilty->setToolTip("Angle of tilt about the Y-Axis (deg)");
    imuStateLayout->addWidget(tilty, 0, 1, 1, 1, Qt::AlignLeft | Qt::AlignBottom );

    a_y = new QLineEdit;
    a_y->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    a_y->setReadOnly(true);
    imuStateLayout->addWidget(a_y, 1, 1, 1, 1, Qt::AlignCenter);

    QLabel* tiltz = new QLabel;
    tiltz->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    tiltz->setText("Tilt Z");
    tiltz->setToolTip("Angle of tilt about the Z-Axis (deg)");
    imuStateLayout->addWidget(tiltz, 0, 2, 1, 1, Qt::AlignLeft | Qt::AlignBottom );

    a_z = new QLineEdit;
    a_z->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    a_z->setReadOnly(true);
    imuStateLayout->addWidget(a_z, 1, 2, 1, 1, Qt::AlignCenter);


    QLabel* velx = new QLabel;
    velx->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    velx->setText("Angular Vel X");
    velx->setToolTip("Angular Velocity about the X-Axis");
    imuStateLayout->addWidget(velx, 3, 0, 1, 1, Qt::AlignLeft | Qt::AlignBottom );

    w_x = new QLineEdit;
    w_x->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    w_x->setReadOnly(true);
    imuStateLayout->addWidget(w_x, 4, 0, 1, 1, Qt::AlignCenter);

    QLabel* vely = new QLabel;
    vely->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    vely->setText("Angular Vel Y");
    vely->setToolTip("Angular Velocity about the Y-Axis");
    imuStateLayout->addWidget(vely, 3, 1, 1, 1, Qt::AlignLeft | Qt::AlignBottom );

    w_y = new QLineEdit;
    w_y->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    w_y->setReadOnly(true);
    imuStateLayout->addWidget(w_y, 4, 1, 1, 1, Qt::AlignCenter);

    QLabel* velz = new QLabel;
    velz->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    velz->setText("Angular Vel X");
    velz->setToolTip("Angular Velocity about the Z-Axis");
    imuStateLayout->addWidget(velz, 3, 2, 1, 1, Qt::AlignLeft | Qt::AlignBottom );

    w_z = new QLineEdit;
    w_z->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    w_z->setReadOnly(true);
    imuStateLayout->addWidget(w_z, 4, 2, 1, 1);

    imuBox->setLayout(imuStateLayout);
    sensorStateLayout->addWidget(imuBox, Qt::AlignHCenter | Qt::AlignTop);

    sensorStateTab = new QWidget;
    sensorStateTab->setLayout(sensorStateLayout);

    std::cerr << "Sensor State Tab complete" << std::endl;

    addTab(commandTab, "Joint Command");
    addTab(jointStateTab, "Joint State");
    addTab(sensorCmdTab, "Sensor Command");
    addTab(sensorStateTab, "Sensor State");




}







} // End hubo_init_space

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( hubo_init_space::HuboInitPanel,rviz::Panel )
PLUGINLIB_EXPORT_CLASS( hubo_init_space::HuboInitWidget, QTabWidget )
