#ifndef UTILS_HPP
#define UTILS_HPP

#include <vector>
#include "Walk.hpp"

#ifndef fp32
typedef float fp32;
#endif

// 获取展示顺序
std::vector<int> getShowOrder(int start, int end);

// 计算两点间距离
fp32 get_distance(Point current, Point target);

// TODO: 自由移动
bool isAutoMove();

#endif // UTILS_HPP