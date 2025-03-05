#include "utils.hpp"
#include <cmath>
#include <vector>

using namespace std;

bool isAutoMove() { // TODO: 自由移动
  return false;
}

fp32 get_distance(Point current, Point target) { //未声明
  return pow(pow(current.x-target.x, 2) + pow(current.y-target.y, 2), 0.5);
}

vector<int> getShowOrder(int start, int end) {
  vector<int> output_ordert = {};
  if (start == end) {
    output_ordert.push_back(start);
    return output_ordert;
  }
  if (start == 3 && end == 6) {
    output_ordert = {3, 2, 6};
    return output_ordert;
  }
  if (start == 6 && end == 3) {
    output_ordert = {6, 2, 3};
    return output_ordert;
  }
  vector<int> current_order1 = {1, 0, 2, 3, 4, 5, 6};
  vector<int> current_order2 = {1, 0, 2, 6, 5, 4, 3};
  vector<int> current_order3 = {6, 5, 4, 3, 2, 0, 1};
  vector<int> current_order4 = {3, 4, 5, 6, 2, 0, 1};
  auto f = [](int start, int end, vector<int> current_order) {
    vector<int> output_order = {};
    bool findStart = false;
    for (int i: current_order) {
      if (i == start) {
        output_order.push_back(i);
        findStart = true;
      }
      else if (findStart && i != end) {
        output_order.push_back(i);
      }
      else if (i == end && findStart) {
        output_order.push_back(i);
        break;
      }
      else if (i == end && !findStart) {
        output_order = {};
        break;
      }

    }
    return output_order;
  };
  vector<vector<int>> nonEmptyVectors;
  if (!f(start, end, current_order1).empty()) {nonEmptyVectors.push_back(f(start, end, current_order1));}
  if (!f(start, end, current_order2).empty()) {nonEmptyVectors.push_back(f(start, end, current_order2));}
  if (!f(start, end, current_order3).empty()) {nonEmptyVectors.push_back(f(start, end, current_order3));}
  if (!f(start, end, current_order4).empty()) {nonEmptyVectors.push_back(f(start, end, current_order4));}
  if (nonEmptyVectors.empty()) {
    return {};
  }

  // cout << "nonEmptyVectors" << nonEmptyVectors.size() << endl;
  // for (auto i: nonEmptyVectors) {
  //   for (auto j: i) {
  //     cout << j << " ";
  //   }
  //   cout << endl;
  // }

  size_t min_vec = 0;
  for (int i = 0; i < nonEmptyVectors.size(); ++i) {
    if (nonEmptyVectors[i].size() < nonEmptyVectors[min_vec].size()) {
      min_vec = i;
    }
  }
  return nonEmptyVectors[min_vec];
}