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

// Description:   Example showing how to use the gait manager
//                and keyboard inputs

#ifndef WALK_HPP
#define WALK_HPP

#define NMOTORS 20

#include <webots/Robot.hpp>

namespace managers {
  class RobotisOp2MotionManager;
  class RobotisOp2GaitManager;
}  // namespace managers

namespace webots {
  class Motor;
  class PositionSensor;
  class LED;
  class Camera;
  class Accelerometer;
  class Gyro;
  class Keyboard;
  class Speaker;
};  // namespace webots


typedef struct Point
{
  float x;
  float y;
}Point;



class Walk : public webots::Robot {
public:
  Walk();
  virtual ~Walk();
  void run();
  void checkIfFallen();
  void RaiseArmToShow(bool &isWalking);
  void Go2Point(Point target_point);
  void GetNowPosition();
  void GetLidarData();

  void Go2PointTangentBug(Point target_point);

  void GetDistanceSensorsValues();

private:
  int mTimeStep;

  void myStep();
  void wait(int ms);

  webots::Motor *mMotors[NMOTORS];
  webots::PositionSensor *mPositionSensors[NMOTORS];
  webots::Accelerometer *mAccelerometer;
  webots::GPS *mGPS;
  webots::InertialUnit *mIMU;
  webots::Keyboard *mKeyboard;
  webots::Lidar *mLidar;
  webots::DistanceSensor *mDistanceSensors[6];

  managers::RobotisOp2MotionManager *mMotionManager;
  managers::RobotisOp2GaitManager *mGaitManager;

  Point now_position;
  float now_yaw;
  const float *lidar_depths;
  float distance_sensors_values[6];
};

#endif
