#include "PathPlanning.hpp"
#include <cmath>
#include <iostream>
#include <limits>

using namespace std;

#ifndef NOT_REVOLVE
#define NOT_REVOLVE std::numeric_limits<float>::infinity()
#endif

//----------Ke's code begin----------

PathPlanning::PathPlanning() {
  PathPlanning::robotStatu = START;
  PathPlanning::current_step = 0;
  PathPlanning::show_order = {};
  key_points = {
    {{3.2, 3.3}, NOT_REVOLVE},
    {{6.2, 3.3}, 0.0f},
    {{3.5, 0.3}, -M_PI/2.0f},
    {{5.8, 0.3}, 0.0f},
    {{5.8, -3.2}, 0.0f},
    {{1.5, -3.2}, M_PI},
    {{1.5, 0.3}, M_PI}
  };
  PathPlanning::controller = new Walk();
}

PathPlanning::PathPlanning(std::vector<int> show_order) {
  PathPlanning::robotStatu = START;
  PathPlanning::current_step = 0;
  PathPlanning::show_order = show_order;
  key_points = {
    {{3.5, 3.3}, NOT_REVOLVE},
    {{6.2, 3.3}, 0.0f},
    {{3.5, 0.3}, -M_PI/2.0f},
    {{5.8, 0.3}, 0.0f},
    {{5.8, -3.2}, 0.0f},
    {{1.5, -3.2}, M_PI},
    {{1.5, 0.3}, M_PI}
  };
  PathPlanning::controller = new Walk();
}

PathPlanning::~PathPlanning() {
  delete PathPlanning::controller;
}

void PathPlanning::showInOrder() {
  cout << "Press the space bar to start/stop walking" << endl;
  cout << "Use the arrow keys to move the robot while walking" << endl;

  // First step to update sensors values
  controller->myStep();
  // play the hello motion
  controller->mMotionManager->playPage(9);  // init position
  controller->wait(200);
  // main loop
  bool isWalking = true;

  bool isWorking = false;
  int current_key = -1;
  int last_current_key = -1;
  int current_p = 0;
  while (true) {
    controller->checkIfFallen();
    controller->GetNowPosition();
    controller->GetDistanceSensorsValues();
    controller->GetLidarData();
    controller->mGaitManager->setXAmplitude(0.0);
    controller->mGaitManager->setAAmplitude(0.0);

    // get keyboard key
    int key = 0;
    while ((key = controller->mKeyboard->getKey()) >= 0) {
      switch (key) {
        case ' ':  // Space bar
          // cout << "keyboard  " << endl;
          if (isWalking) {
            controller->mGaitManager->stop();
            isWalking = false;
            controller->wait(200);
          } else {
            controller->mGaitManager->start();
            isWalking = true;
            controller->wait(200);
          }
          break;
        case 'G':
          isWorking = true;
          break;
        case 'S':
          isWorking = false;
          break;
        case '0':
          current_key = 0;
          break;
        case '1':
          current_key = 1;
          break;
        case '2':
          current_key = 2;
          break;
        case '3':
          current_key = 3;
          break;
        case '4':
          current_key = 4;
          break;
        case '5':
          current_key = 5;
          break;
        case '6':
          current_key = 6;
          break;
        // case 'T':
        //   current_key = 10;
          break;
        default:
          break;
      }
    }
    if (controller->mKeyboard->getKey() == -1)  //没有键盘键入
    {
      // cout<<"current_key: "<<current_key<<endl;
      // cout<<"current_p: "<<current_p<<endl;
      if (isWorking) 
      {
        if (current_key != last_current_key || current_key == -1) 
        {
          int current_point = 0;
          float current_distance = 1000.0f;
          for (int i = 0; i <= 6; ++i) {
            if (get_distance(controller->now_position, key_points[i].p) < current_distance) {
              current_point = i;
              current_p = i;
              current_distance = get_distance(controller->now_position, key_points[i].p);
            }
          }
          vector<int> show_order = getShowOrder(current_point, current_key);
          // controller->wait(200);

          // for (int i: show_order) {
          //   cout << i << " ";
          // }
          // cout << endl;

          if (!show_order.empty()) 
          {
            PathPlanning::show_order = show_order;
            PathPlanning::robotStatu = START;
            current_step = 0;
          }
          last_current_key = current_key;
          current_key = -1;
        }
        // cout << "keyboard g" << endl;
        switch (PathPlanning::robotStatu)
        {
        case RobotStatu_e::START:
          if (current_step < int(show_order.size())) {
            PathPlanning::robotStatu = RobotStatu_e::RUNNING;
          }
          else {
            PathPlanning::robotStatu = RobotStatu_e::OFF;
          }
          break;
        case RobotStatu_e::RUNNING:
          // cout << "RUNNING" << endl;
          controller->GetNowPosition();
          if (get_distance(controller->now_position, key_points[show_order[current_step]].p) < 0.2) {
            PathPlanning::robotStatu = RobotStatu_e::REVOLVE;
          }
          else {
            controller->Go2PointBug0(key_points[show_order[current_step]].p);
          }
          if (isAutoMove()) {
            PathPlanning::robotStatu = RobotStatu_e::AUTO_MOVE;
          }
          break;

        case RobotStatu_e::AUTO_MOVE:
          if (!isAutoMove()) {
            PathPlanning::robotStatu = RobotStatu_e::RUNNING;
          }
          break;

        case RobotStatu_e::REVOLVE:
          // 修改类型比较的地方
          if ((fabs(controller->now_yaw - key_points[show_order[current_step]].yaw) < 0.1 || 
               fabs(fabs(controller->now_yaw - key_points[show_order[current_step]].yaw) - 2*M_PI) < 0.1 ) || 
               key_points[show_order[current_step]].yaw == NOT_REVOLVE || 
               current_step != static_cast<int>(show_order.size() - 1)) {
            if ((current_step == static_cast<int>(show_order.size() - 1)) && show_order[current_step] != 0) {
              PathPlanning::robotStatu = RobotStatu_e::SHOW;
            }
            
            else {
              if (show_order[current_step] != 0) {
                current_step++;
                PathPlanning::robotStatu = RobotStatu_e::START;
              }
              else {
                current_step++;
                PathPlanning::robotStatu = RobotStatu_e::START;
              }       
            }

          }
          else {
            controller->RevolveYaw(key_points[show_order[current_step]].yaw);
          }
          break;
        case RobotStatu_e::SHOW:
            // cout << "SHOW" << endl;
            controller->RaiseArmToShow(isWalking);
            current_step++;
            PathPlanning::robotStatu = RobotStatu_e::START;
            controller->wait(1000);
            controller->mGaitManager->start();
            isWalking = true;
            controller->wait(200);
          break;
        case RobotStatu_e::OFF:
          break;
        default:
          break;
        }
      }
    }
    controller->mGaitManager->step(controller->mTimeStep);

    // step
    controller->myStep();
    
    if (isWorking) {
      if (current_key != -1) {
        if ((current_step >= static_cast<int>(show_order.size() - 1)) && PathPlanning::robotStatu == RobotStatu_e::OFF) {
          cout << "已到达" << show_order[show_order.size()-1] << "号展品"<< endl;
        }
        else {
        cout << "出发坐标: (" << key_points[current_p].p.x << "," << key_points[current_p].p.y << ")" << 
        " 目标坐标: (" << key_points[current_key].p.x << "," << key_points[current_key].p.y << ")" << endl;
        }
        if (show_order[show_order.size()-1] != 0) {
          cout << "正在前往展品："<<show_order[show_order.size()-1] << "号展品" <<endl;
        }
        else {
          cout << "返回0号点位"<< endl;
        }
      }
      else if (current_key == -1) {
        cout << "请选择目标展品" << endl;
      }

    }
    else {
      cout << "请按G键开启导航" << endl;
    }




  }


}
//----------Ke's code end----------