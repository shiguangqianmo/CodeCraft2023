#pragma once
#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <queue>

#include "Worktop.h"
#include "base.h"
using namespace std;

class Robot {
public:
	Robot(double point_x, double point_y);
	bool updata_param(int w_i, int p, double t_c, double c_c, double o, double vx, double vy, double d, double point_x, double point_y);
	~Robot();
	pair<string, pair<int, float>> RotateControl(vector<Worktop*>&, int);
	pair<string, pair<int, float>> VelControl(vector<Worktop*>&, int);
	void RotateAndVelControl(vector<pair<string, pair<int, float>>>&, vector<Worktop*>&, vector<Robot*>&, int, int);
	int CheckCollide(vector<Robot*>&, int);
	bool GetAssign(vector<vector<int>>& task_link);
	bool is_legal(double time, pair<int, int>&);
	void updata_used_task_pair(vector<int>& assign);

private:
	inline double getSellPrice(int price, double time_coef, double coll_coef) { return static_cast<double>(price) * time_coef * coll_coef; };
	inline double Dist(double r_x, double r_y, double w_x, double w_y) { return sqrt((r_x - w_x) * (r_x - w_x) + (r_y - w_y) * (r_y - w_y)); };

public:
	int work_id;                  // 所处工作台ID，[-1，工作台总数-1]
	int product;                  // 携带物品类型，0：未携带
	double time_coef, coll_coef;  // 时间、碰撞价值系数[0.9, 1]
	double omega;                 // 角速度，>0：逆时针，<0：顺时针
	double v_x, v_y;              // 线速度
	double direction;             // 朝向[-pi, pi]
	double x, y;                  // 坐标
	int bind;                     // 绑定工作台的ID，-1表示未绑定任何工作台
	vector<int> assignment;		  // 任务链 eg:{1, 4, 7, 8}
};

constexpr double value_coef = -500;	// 机器人到源工作台的价值系数
constexpr double min_value = value_coef * 100;	// 最小价值（距离不会超过100）
extern vector<Worktop*> works;		// 若干个工作台
extern vector<vector<double>> time_table;	// 50 * 50的时间表
extern vector<vector<double>> value_w2w;	// K * K的价值表
//extern deque<pair<int, int>> assign_list;	// 任务清单
extern vector<int> resources;			// 场上资源
extern vector<pair<int, int>> used_task_pair;	// 被占用的任务对，读帧数据时更新
