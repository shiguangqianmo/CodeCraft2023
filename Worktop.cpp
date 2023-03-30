#include "Worktop.h"

// 构造函数，初始化一个工作台状态
Worktop::Worktop(int t, double point_x, double point_y) {
	type = t;
	x = point_x;
	y = point_y;
	remain_frames = (type < 4 ? 50 : -1);	// 初始化的时候给1，2，3工作台赋予剩余帧，方便给机器人分配初始任务
	raw = 0;
	product = 0;
	value = 0;
}

// 根据读取的帧数据进行状态更新
bool Worktop::updata_param(int t, double point_x, double point_y, int remain, int r, int p) {
	type = t;
	x = point_x;
	y = point_y;
	remain_frames = remain;
	raw = r;
	product = p;
	value = this->calc_value(raw);
	return true;
}


// 该工作台的这类（carried_type）原料格是否已经被占用
// 被占用了返回false，空着返回true		******** 函数名是否要改一下
bool Worktop::isUsed(int carried_type) {
	return ((1 << carried_type) & raw) == 0;
}


// 计算工作台上所有物品的价值（利润），包括潜在产品
double Worktop::calc_value(int raw) {
	if (type == 8)
		return value_8;
	if (type == 9)
		return value_9;
	if (type == 7 && !product && remain_frames < 0)
		return 3400 * 2;	// 如果7号台没有产品和潜在产品，赋予7一个与3号台相当的价值

	double ans = 0;
	if (product)	// 如果有产品，加上产品价值（利润）
		ans += value_table[type - 1][1] - value_table[type - 1][0];
	if(remain_frames > 0)	// 如果正在生产产品，再加一个产品利润	（3.21新增）
		ans += value_table[type - 1][1] - value_table[type - 1][0];

	//int i = 0;
	//while (raw) {	// 原料价值
	//	if (raw & 1) ans += value_table[i - 1][1] - value_table[i - 1][0];
	//	raw >>= 1;
	//	i++;
	//}
	while (raw) {
		ans += (raw & 1);
		raw >>= 1;
	}
	return ans;
}


Worktop::~Worktop() {}

