#ifndef PATH_PLANNING_HPP
#define PATH_PLANNING_HPP

#include "Walk.hpp"
#include "utils.hpp"
#include <RobotisOp2GaitManager.hpp>
#include <RobotisOp2MotionManager.hpp>
#include <webots/Keyboard.hpp>
#include <vector>

class PathPlanning {
private:
    RobotStatu_e robotStatu;
    int current_step;
    Walk *controller;
    // 已移除 std::vector<PointWithYaw> key_points;

public:
    PathPlanning();
    PathPlanning(std::vector<int> show_order);
    ~PathPlanning();
    void showInOrder();
    std::vector<int> show_order;
};

#endif // PATH_PLANNING_HPP