#pragma once
#include <iostream>
#include <vector>
#include <algorithm>
using namespace std;

class Worktop {
public:
	Worktop(int t, double x, double y);
	bool updata_param(int t, double x, double y, int remain, int r, int p);
	~Worktop();
	bool isUsed(int);
	double calc_value(int raw);		// 计算工作台上所有物品的价值

public:
	int type;           // 工作台类型（int, [1, 9]）
	double x, y;        // 坐标
	int remain_frames;  // 剩余生产帧数（-1：没生产，0：阻塞，>=0：剩余生产帧数）
	int raw;            // 原材料格状态
	int product;        // 产品格状态，0无1有
	double value;			// 该工作台上所有物品价值
};

constexpr double value_8 = 0;		
constexpr double value_9 = 0;		// 8、9号工作台的价值（利润）
extern int value_table[7][2];
