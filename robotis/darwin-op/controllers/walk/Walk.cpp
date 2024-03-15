// Copyright 1996-2022 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
When running this controller in the real robot
do not interface via SSh client (i,e, PuTTY)
The result will be on a segmentation fault error.

Instead interface with the robot via remote desktop
(i.e. VNC). Open a terminal window, compile controller
and run.
*/
#include "Walk.hpp"

#include <RobotisOp2GaitManager.hpp>
#include <RobotisOp2MotionManager.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Gyro.hpp>
#include <webots/Keyboard.hpp>
#include <webots/LED.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/GPS.hpp>
#include <webots/InertialUnit.hpp>

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>

using namespace webots;
using namespace managers;
using namespace std;

#define M_PI 3.14159265358979323846

static const char *motorNames[NMOTORS] = {
  "ShoulderR" /*ID1 */, "ShoulderL" /*ID2 */, "ArmUpperR" /*ID3 */, "ArmUpperL" /*ID4 */, "ArmLowerR" /*ID5 */,
  "ArmLowerL" /*ID6 */, "PelvYR" /*ID7 */,    "PelvYL" /*ID8 */,    "PelvR" /*ID9 */,     "PelvL" /*ID10*/,
  "LegUpperR" /*ID11*/, "LegUpperL" /*ID12*/, "LegLowerR" /*ID13*/, "LegLowerL" /*ID14*/, "AnkleR" /*ID15*/,
  "AnkleL" /*ID16*/,    "FootR" /*ID17*/,     "FootL" /*ID18*/,     "Neck" /*ID19*/,      "Head" /*ID20*/
};

Walk::Walk() : Robot() {
  mTimeStep = getBasicTimeStep();

  getLED("HeadLed")->set(0xFF0000);
  getLED("EyeLed")->set(0x00FF00);
  mAccelerometer = getAccelerometer("Accelerometer");
  mGPS = getGPS("gps");
  mIMU = getInertialUnit("imu");
  mAccelerometer->enable(mTimeStep);

  getGyro("Gyro")->enable(mTimeStep);

  for (int i = 0; i < NMOTORS; i++) {
    mMotors[i] = getMotor(motorNames[i]);
    string sensorName = motorNames[i];
    sensorName.push_back('S');
    mPositionSensors[i] = getPositionSensor(sensorName);
    mPositionSensors[i]->enable(mTimeStep);
  }

  mKeyboard = getKeyboard();
  mKeyboard->enable(mTimeStep);
  mGPS->enable(mTimeStep);
  mIMU->enable(mTimeStep);

  mMotionManager = new RobotisOp2MotionManager(this);
  mGaitManager = new RobotisOp2GaitManager(this, "config.ini");
}

/*          x
            |
            |
            |
  y<-----------------   逆时针 yaw角增大，初始为0
            |
            |  
*/

void Walk::GetNowPosition(){
  const double *position = mGPS->getValues();
  now_position.x = position[0];
  now_position.y = position[1];
  now_yaw = mIMU->getRollPitchYaw()[2];
  // cout<<"yaw: "<<now_yaw<<endl;
  // cout<<"x: "<<now_position.x<<" y: "<<now_position.y<<endl;
}

void Walk::Go2Point(Point target_point){
  // get the current position of the robot
  double x = now_position.x;
  double y = now_position.y;
  double theta = now_yaw;
  double x_target = target_point.x;
  double y_target = target_point.y;
  double theta_target = atan2(y_target - y, x_target - x);
  double distance = sqrt(pow(x_target - x, 2) + pow(y_target - y, 2));
  double angle = theta_target - theta;
  if (angle > M_PI)
    angle -= 2 * M_PI;
  if (angle < -M_PI)
    angle += 2 * M_PI;

  // cout<<"distance: "<<distance<<" angle: "<<angle<<endl;
  if (distance > 0.1) {
    mGaitManager->setXAmplitude(1.0);
    mGaitManager->setAAmplitude(angle);
  } else {
    mGaitManager->setXAmplitude(0.0);
    mGaitManager->setAAmplitude(0.0);
  }
}

Walk::~Walk() {
}

void Walk::myStep() {
  int ret = step(mTimeStep);
  if (ret == -1)
    exit(EXIT_SUCCESS);
}

void Walk::wait(int ms) {
  double startTime = getTime();
  double s = (double)ms / 1000.0;
  while (s + startTime >= getTime())
    myStep();
}


void Walk::RaiseArmToShow(bool &isWalking){
    if (isWalking) {
      mGaitManager->stop();
      mMotors[2]->setPosition(-0.68);
      mMotors[4]->setPosition(-1.65);
      mMotors[0]->setPosition(2.3);
      wait(200);
    } else {
      mGaitManager->start();
      isWalking = true;
      wait(200);
    }
}


// function containing the main feedback loop
void Walk::run() {
  cout << "-------Walk example of ROBOTIS OP2-------" << endl;
  cout << "This example illustrates Gait Manager" << endl;
  cout << "Press the space bar to start/stop walking" << endl;
  cout << "Use the arrow keys to move the robot while walking" << endl;

  // First step to update sensors values
  myStep();

  // play the hello motion
  mMotionManager->playPage(9);  // init position
  wait(200);

  // main loop
  bool isWalking = false;

  while (true) {
    checkIfFallen();
    GetNowPosition();

    mGaitManager->setXAmplitude(0.0);
    mGaitManager->setAAmplitude(0.0);

    // get keyboard key
    int key = 0;
    while ((key = mKeyboard->getKey()) >= 0) {
      switch (key) {
        case ' ':  // Space bar
          if (isWalking) {
            mGaitManager->stop();
            isWalking = false;
            wait(200);
          } else {
            mGaitManager->start();
            isWalking = true;
            wait(200);
          }
          break;
        case Keyboard::UP:
          mGaitManager->setXAmplitude(1.0);
          break;
        case Keyboard::DOWN:
          mGaitManager->setXAmplitude(-1.0);
          break;
        case Keyboard::RIGHT:
          mGaitManager->setAAmplitude(-0.5);
          break;
        case Keyboard::LEFT:
          mGaitManager->setAAmplitude(0.5);
          break;
        case 'Q':
          RaiseArmToShow(isWalking);
          break;
        case 'K':
          Go2Point({3.52, 2.2});
          break;
      }
    }

    mGaitManager->step(mTimeStep);

    // step
    myStep();
  }
}

void Walk::checkIfFallen() {
  static int fup = 0;
  static int fdown = 0;
  static const double acc_tolerance = 80.0;
  static const double acc_step = 100;

  // count how many steps the accelerometer
  // says that the robot is down
  const double *acc = mAccelerometer->getValues();
  if (acc[1] < 512.0 - acc_tolerance)
    fup++;
  else
    fup = 0;

  if (acc[1] > 512.0 + acc_tolerance)
    fdown++;
  else
    fdown = 0;

  // the robot face is down
  if (fup > acc_step) {
    mMotionManager->playPage(10);  // f_up
    mMotionManager->playPage(9);   // init position
    fup = 0;
  }
  // the back face is down
  else if (fdown > acc_step) {
    mMotionManager->playPage(11);  // b_up
    mMotionManager->playPage(9);   // init position
    fdown = 0;
  }
}
