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
#include <webots/Lidar.hpp>
#include <webots/DistanceSensor.hpp>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>

#include <algorithm>

#include <utils.hpp>

using namespace webots;
using namespace managers;
using namespace std;

#define M_PI 3.14159265358979323846

#ifndef NOT_REVOLVE
#define NOT_REVOLVE std::numeric_limits<float>::infinity()
#endif

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
  mLidar = getLidar("Sick LMS 291");
  mDistanceSensors[0] = getDistanceSensor("front1");
  mDistanceSensors[1] = getDistanceSensor("front2");
  mDistanceSensors[2] = getDistanceSensor("left1");
  mDistanceSensors[3] = getDistanceSensor("left2");
  mDistanceSensors[4] = getDistanceSensor("right1");
  mDistanceSensors[5] = getDistanceSensor("right2");
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
  mLidar->enable(mTimeStep);
  mLidar->enablePointCloud();

  for (int i = 0; i < 6; i++)
    mDistanceSensors[i]->enable(mTimeStep);

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

float thre = 4.5;

void Walk::Go2PointBug0Improved(Point target_point) {
  // 获取机器人当前位置
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

  // 如果角度较大且前方无障碍物，直接前往目标点
  if (fabs(angle) > M_PI/6 && lidar_depths[90] > 0.8) {
    Go2Point(target_point);
    return;
  }

  // 检测障碍物左侧边缘
  int left_edge = -1;
  for (int j = 45; j < 135; j++) {
    if (lidar_depths[j+1] > 2.0) {
      continue;
    }
    if (lidar_depths[j] - lidar_depths[j+1] > 0.8) {
      left_edge = j+1;
      break;
    }
  }

  // 检测障碍物右侧边缘
  int right_edge = -1;
  for (int j = 135; j > 45; j--) {
    if (lidar_depths[j-1] > 2.0) {
      continue;
    }
    if (lidar_depths[j] - lidar_depths[j-1] > 0.8) {
      right_edge = j-1;
      break;
    }
  }

  // 如果没有检测到任何障碍物，直接前往目标点
  if (left_edge == -1 && right_edge == -1) {
    Go2Point(target_point);
    return;
  }

  float x_left_obstacle = 0.0;
  float y_left_obstacle = 0.0;
  float min_left_distance = 100.0;
  
  // 如果检测到左侧边缘
  if (left_edge != -1) {
    // 计算障碍物左侧边缘点的坐标
    x_left_obstacle = x + lidar_depths[left_edge] * sin(left_edge * M_PI / 180 + now_yaw);
    y_left_obstacle = y + lidar_depths[left_edge] * cos(left_edge * M_PI / 180 + now_yaw);

    // 计算左侧空间 - 使用欧式距离
    for (int j = left_edge - 15; j < left_edge; j++) {
      if (j >= 0 && j < 180) {
        // 计算当前点的坐标
        float x_point = x + lidar_depths[j] * sin(j * M_PI / 180 + now_yaw);
        float y_point = y + lidar_depths[j] * cos(j * M_PI / 180 + now_yaw);
        
        // 计算当前点到左侧边缘点的欧式距离
        float dist = sqrt(pow(x_point - x_left_obstacle, 2) + pow(y_point - y_left_obstacle, 2));
        min_left_distance = min(min_left_distance, dist);
      }
    }
  }

  float x_right_obstacle = 0.0;
  float y_right_obstacle = 0.0;
  
  // 如果检测到右侧边缘
  if (right_edge != -1) {
    x_right_obstacle = x + lidar_depths[right_edge] * sin(right_edge * M_PI / 180 + now_yaw);
    y_right_obstacle = y + lidar_depths[right_edge] * cos(right_edge * M_PI / 180 + now_yaw);
  }

  // 判断障碍物是否在目标点前方
  int obstacled = 0;
  int ratio1 = 1;
  float temp_yaw = now_yaw + M_PI/2;
  if (temp_yaw > M_PI) {
    temp_yaw -= 2 * M_PI;
  }
  if (temp_yaw < -M_PI) {
    temp_yaw += 2 * M_PI;
  }

  // 判断障碍物是否在目标点前方
  // 优先使用左侧边缘判断，如果没有左侧边缘则使用右侧边缘
  float x_obstacle = (left_edge != -1) ? x_left_obstacle : x_right_obstacle;
  float y_obstacle = (left_edge != -1) ? y_left_obstacle : y_right_obstacle;

  if (temp_yaw > M_PI/3 && temp_yaw < 2 * M_PI/3) {
    if (x_obstacle < x_target) {
      obstacled = 1;
      ratio1 = 1;
    }
  } else if (temp_yaw > -2 * M_PI/3 && temp_yaw < -M_PI/3) {
    if (x_obstacle > x_target) {
      obstacled = 1;
      ratio1 = -1;
    }
  } else if (temp_yaw > -M_PI/3 && temp_yaw < M_PI/3) {
    if (y_obstacle > y_target) {
      obstacled = 1;
      ratio1 = -1;
    }
  } else {
    if (y_obstacle < y_target) {
      obstacled = 1;
      ratio1 = 1;
    }
  }

  if (obstacled) {
    // 决定绕行方向
    // 1. 如果只检测到左侧边缘，从左侧绕行
    // 2. 如果只检测到右侧边缘，从右侧绕行
    // 3. 如果两侧都检测到，且左侧空间小于1.0，从右侧绕行
    // 4. 否则从左侧绕行
    if (left_edge == -1 && right_edge != -1) {
      // 只检测到右侧边缘，从右侧绕行
      Go2Point({x_right_obstacle + thre*ratio1, y_right_obstacle - thre*ratio1});
      printf("只检测到右侧边缘，从右侧绕行\n");
    } else if (left_edge != -1 && (right_edge == -1 || min_left_distance >= 1.0)) {
      // 只检测到左侧边缘或左侧空间足够，从左侧绕行
      Go2Point({x_left_obstacle - thre*ratio1, y_left_obstacle + thre*ratio1});
      printf("从左侧绕行，左侧最小距离: %f\n", min_left_distance);
    } else if (left_edge != -1 && right_edge != -1 && min_left_distance < 1.0) {
      // 两侧都检测到且左侧空间不足，从右侧绕行
      Go2Point({x_right_obstacle + thre*ratio1, y_right_obstacle - thre*ratio1});
      printf("左侧空间不足，从右侧绕行，左侧最小距离: %f\n", min_left_distance);
    }
  } else {
    // 障碍物在目标点后方，直接前往目标点
    Go2Point(target_point);
  }
}

void Walk::Go2PointBug0(Point target_point){
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




  // 基于激光雷达的深度信息，判断是否有障碍物
  // 有障碍物，就绕行
  // 没有障碍物，直接走向目标点

  // 1. 基于激光雷达的深度信息，发现深度突变的地方，就是障碍物的边缘
  int i=-1;

  for(int j=45; j<135; j++){

    if(lidar_depths[j+1] > 2.0){
      continue;
    }
    if(lidar_depths[j] - lidar_depths[j+1] > 0.8){
      // 有障碍物
      i = j+1; // 记录障碍物左侧边缘的角度
      break;

  }


  }

  if(fabs(angle) > M_PI/6 && lidar_depths[i] > 0.8){
    Go2Point(target_point);
    return;
  }

  

  if (i != -1)
  {
    // cout << "障碍物角度: " << i << endl;
    // cout<<"障碍物深度: "<<lidar_depths[i]<<endl;
    // cout<<"yaw: "<<now_yaw<<endl;
    
    // cout<<"x: "<<x<<" y: "<<y<<endl;
    // 2. 计算障碍物的边缘点的坐标
    float x_obstacle = x + lidar_depths[i] * sin(i * M_PI / 180 + now_yaw);
    float y_obstacle = y + lidar_depths[i] * cos(i * M_PI / 180 + now_yaw);


    // cout<<"目标角度:"<<theta_target<<endl;

    // 3. 检查障碍物边缘点是否在目标点的前方
    // 如果在目标点的前方，就绕行
    // 如果在目标点的后方，就直接走向目标点
    int obstacled = 0;
    int ratio1 = 1;
    float temp_yaw = now_yaw + M_PI/2;
    if(temp_yaw > M_PI){
      temp_yaw -= 2 * M_PI;
    }
    if(temp_yaw < -M_PI){
      temp_yaw += 2 * M_PI;
    }

    // cout<<"temp_yaw: "<<temp_yaw<<endl;

    if(temp_yaw > M_PI/3 && temp_yaw < 2 * M_PI/3){
        if(x_obstacle < x_target){
          // cout<<"障碍物在目标点的前方"<<endl;
          obstacled = 1;
          ratio1 = 1;
      }
    }
    else if(temp_yaw > -2 * M_PI/3 && temp_yaw < -M_PI/3){
        if(x_obstacle > x_target){
          // cout<<"障碍物在目标点的前方"<<endl;
          obstacled = 1;
          ratio1 = -1;
      }
    }
    else if(temp_yaw > -M_PI/3 && temp_yaw < M_PI/3){
        if(y_obstacle > y_target){
          // cout<<"障碍物在目标点的前方"<<endl;
          obstacled = 1;
          ratio1 = -1;
      }
    }
    else{
        if(y_obstacle < y_target){
          // cout<<"障碍物在目标点的前方"<<endl;
          obstacled = 1;
          ratio1 = 1;
      }
    }




    // if (x_obstacle * sin(theta_target + 1.5708) - x_target * sin(theta_target + 1.5708) + 
    //       y_target * cos(theta_target + 1.5708) - y_obstacle * cos(theta_target + 1.5708) < 0)
    if(obstacled)
    {
      // 障碍物在目标点的前方
      // 绕行
      // cout << "绕行" << endl;
      // cout<<"ratio1:"<<ratio1<<endl;
      // cout<<"x_obstacle: "<<x_obstacle<<" y_obstacle: "<<y_obstacle<<endl;
      Go2Point({x_obstacle - thre*ratio1, y_obstacle + thre*ratio1});
    }
    else
    {
      // 障碍物在目标点的后方
      // 直接走向目标点
      Go2Point(target_point);
    }

  }
  else
  {
    // 没有障碍物
    // 直接走向目标点
    Go2Point(target_point);
  }

}

void Walk::RevolveYaw(fp32 target_yaw)
{
  if (target_yaw == NOT_REVOLVE) {
    return;
  }
  double angle = target_yaw - now_yaw;
  if (angle > M_PI)
    angle -= 2 * M_PI;
  if (angle < -M_PI)
    angle += 2 * M_PI;
  mGaitManager->setXAmplitude(0.0);
  mGaitManager->setAAmplitude(0.5*angle);
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



void Walk::GetDistanceSensorsValues()
{
  for (int i = 0; i < 6; i++)
  {
    distance_sensors_values[i] = mDistanceSensors[i]->getValue();
  }

  // cout<<"Distance Sensor 0: "<<distance_sensors_values[0]<<endl;
  // cout<<"Distance Sensor 1: "<<distance_sensors_values[1]<<endl;

}


void Walk::GetLidarData(){
  lidar_depths = mLidar->getRangeImage();

  // cout<<"Lidar Depth 90 degree: "<<lidar_depths[90]<<endl;
  // cout<<"Lidar getNumberOfLayers: "<<mLidar->getNumberOfLayers()<<endl;  // 1
  // cout<<"Lidar Depth Number: "<<sizeof(lidar_depths)/sizeof(lidar_depths[0])<<endl;
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
          RaiseArmToShow(isWalking); // isWalking = True -> False
          break;
        case 'K':
          Go2Point({3.52, 2.2}); // isWalking = True
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

