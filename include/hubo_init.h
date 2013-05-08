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

#ifndef HUBO_INIT_H
#define HUBO_INIT_H

#include <ros/ros.h>
#include <QTableWidget>
#include <QPushButton>
#include <QLineEdit>
#include <QRadioButton>
#include <QSpinBox>
#include <QLabel>
#include <QProcess>
#include <QGroupBox>
#include <QButtonGroup>

#include <vector>

#include <rviz/panel.h>

#include <hubo.h>

namespace hubo_init_space
{

// Here we declare our new subclass of rviz::Panel.  Every panel which
// can be added via the Panels/Add_New_Panel menu is a subclass of
// rviz::Panel.




class HuboInitWidget: public QTabWidget
{
// This class uses Qt slots and is a subclass of QObject, so it needs
// the Q_OBJECT macro.
Q_OBJECT
public:
  // QWidget subclass constructors usually take a parent widget
  // parameter (which usually defaults to 0).  At the same time,
  // pluginlib::ClassLoader creates instances by calling the default
  // constructor (with no arguments).  Taking the parameter and giving
  // a default of 0 lets the default constructor work and also lets
  // someone using the class for something else to pass in a parent
  // widget as they normally would with Qt.
  HuboInitWidget( QWidget* parent = 0 );
  ~HuboInitWidget();

  // Handler for the nested ach daemon process
  QProcess achd;

  // Ach Channels for sending and receiving data
  ach_channel_t stateChan;
  ach_channel_t cmdChan;

  // Structs for storing data to transmit
  hubo_state_t state;
  hubo_board_cmd_t cmd;

  void initializeAch();

  void normifyButton(const QPushButton* button); // Used if state is normal
  void purpifyButton(const QPushButton* button); // Used if joint is homing/failed to home
  void reddifyButton(const QPushButton* button); // Used if joint has an error
  void yellifyButton(const QPushButton* button); // Used if joint is inactive


  // Slots will be "connected" to signals in order to respond to user events
protected Q_SLOTS:

  // Detect which button is being pressed
  void detectJointCmdButton(int id);
  void handleJointStateButton(int id);
  void detectSensorCmdButton(int id);

  // Send the command once the joint button is released
  void handleJointCmdButton(int id);
  void handleJointStateButton(int id);

  // Send sensor commands once the button is released
  void handleSensorButton(int id);

  void handleHomeAll();
  void handleHomeBad();
  void handleInitSensors();

  // Update all state information
  void refreshState();

  // Deal with achd crashes/failures
  void achdExit();

private:

  ///////////////
  QWidget* commandTab;

    QPushButton* achdConnect;

    QPushButton* homeAll;
    QPushButton* homeBad;
    QPushButton* initSensors;
    QSpinBox*    refreshRate;

    // Vector of buttons for the different joints
    QButtonGroup* jointCmdGroup;
    std::vector<QPushButton*> jointCmdButtons;

    QButtonGroup* radioCmdButtons;
    QRadioButton* home;
    QRadioButton* reset;
    QRadioButton* ctrlOn;
    QRadioButton* ctrlOff;
    QRadioButton* fetOn;
    QRadioButton* fetOff;
    QRadioButton* zero;
    QRadioButton* initJoint; // Note: Not used... feels dangerous
  ///////////////


  ///////////////
  QWidget* jointStateTab;

    QLineEdit stateFlags;

    std::vector<QPushButton*> jointStateButtons;
  ///////////////

  std::vector<QString> ftName;


  ///////////////
  QWidget* sensorCmdTab;

    QRadioButton* nullSensor;
    QRadioButton* zeroSensor;
    QRadioButton* initSensor;
  ///////////////


  ///////////////
  QWidget* sensorStateTab;

    std::vector<QLineEdit*> ft_mx;
    std::vector<QLineEdit*> ft_my;
    std::vector<QLineEdit*> ft_fz;

    QLineEdit* a_x;
    QLineEdit* a_y;
    QLineEdit* a_z;

    QLineEdit* w_x;
    QLineEdit* w_y;
    QLineEdit* w_z;
  ///////////////

};

class HuboInitPanel: public rviz::Panel
{
Q_OBJECT
public:
    HuboInitPanel(QWidget *parent = 0);

    // Now we declare overrides of rviz::Panel functions for saving and
    // loading data from the config file.  Here the data is the
    // topic name.
//    virtual void load( const rviz::Config& config );
//    virtual void save( rviz::Config config ) const;

private:

    HuboInitWidget* content;

};

} // end namespace rviz_plugin_tutorials


#endif // TELEOP_PANEL_H
