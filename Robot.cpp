#include "Robot.h"

// 构造函数，初始化一个机器人状态
Robot::Robot(double point_x, double point_y) {
	work_id = -1;
	product = 0;
	time_coef = coll_coef = 0.0;
	omega = 0.0;
	v_x = v_y = 0.0;
	direction = 0.0;
	x = point_x;
	y = point_y;
	bind = -1;					// 初始状态未绑定任何工作台
	assignment = { -1, -1 };	// 初始化还没有任务
}


// 根据读取的帧数据进行状态更新
bool Robot::updata_param(int w_i, int p, double t_c, double c_c, double o, double vx, double vy, double d, double point_x, double point_y) {
	work_id = w_i;
	product = p;
	time_coef = t_c;
	coll_coef = c_c;
	omega = o;
	v_x = vx;
	v_y = vy;
	direction = d;
	x = point_x;
	y = point_y;
	return true;
}


pair<string, pair<int, float>> Robot::RotateControl(vector<Worktop*>& works, int robotID) {
	
}



pair<string, pair<int, float>> Robot::VelControl(vector<Worktop*>& works, int robotID) {
	
}



void Robot::RotateAndVelControl(vector<pair<string, pair<int, float>>>& manipulations, vector<Worktop*>& works, vector<Robot*>& robots, int robotId,
	int state) {
	
}


int Robot::CheckCollide(vector<Robot*>& robots, int robotID) {
	
}


// 从任务清单中选择一个价值最高的任务
bool Robot::GetAssign(int priority_num) {
	double max_value = min_value, cur_value = 0;
	pair<int, int> max_task, cur_task;

	if (priority_num == 0) priority_num = assign_list.size();	// 如果没有优先任务，则全都为备选任务
	while (priority_num--) {
		cur_task = assign_list.front();
		assign_list.pop_front();

		int srcId = cur_task.first, tarId = cur_task.second;
		// 机器人到源工作台的价值
		double dist;
		if (this->work_id == srcId)	// 机器人在源工作台附近
			dist = 2 * MAX_FORWARD_VEL * (time_per_frame * works[srcId]->remain_frames);	// 2vt
		else dist = calc_dist(this->x, this->y, works[srcId]->x, works[srcId]->y);		// d
		cur_value = value_coef * dist;
		cur_value += works[srcId]->value + works[tarId]->value;		// 工作台上的物品价值
		cur_value += value_w2w[srcId][tarId];		// 源工作台到目标工作台的价值

		if (cur_value > max_value) {
			max_value = cur_value;
			max_task = cur_task;
		}
	}
	if (max_value > min_value) {		// 成功分配任务
		this->assignment = max_task;
		return true;
	}

	return false;	
}

Robot::~Robot() {}
