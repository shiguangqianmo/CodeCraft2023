#pragma once
#include <unordered_map>
#include <vector>
using namespace std;

constexpr double time_per_frame = 0.02;         // 每帧的时间
constexpr int frequency = 50;                   // 帧率
constexpr double MAX_FORWARD_VEL = 6.0;
constexpr int MAX_K = 50;

// 每种工作台的购买价[type-1][0]和售出价[type-1][1]
extern int value_table[7][2];
// 工作台收购产品表，即根据机器人所持有的product映射到对应工作台
extern unordered_map<int, vector<int>> next_ws;
// 枚举类，表示操作
enum MANIPULATION { forward = 1, rotate, buy, sell, destroy };
extern unordered_map<string, int> manipulation_dict;

// 计算两点之间的距离
double calc_dist(double x1, double y1, double x2, double y2);

//计算时间价值系数
double calc_time_coef(double pocess_frame, int max_lost);