#include "Robot.h"

// ���캯������ʼ��һ��������״̬
Robot::Robot(double point_x, double point_y) {
	work_id = -1;
	product = 0;
	time_coef = coll_coef = 0.0;
	omega = 0.0;
	v_x = v_y = 0.0;
	direction = 0.0;
	x = point_x;
	y = point_y;
	bind = -1;					// ��ʼ״̬δ���κι���̨
	assignment = { -1, -1 };	// ��ʼ����û������
}


// ���ݶ�ȡ��֡���ݽ���״̬����
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


// �������嵥��ѡ��һ����ֵ��ߵ�����
bool Robot::GetAssign(int priority_num) {
	double max_value = min_value, cur_value = 0;
	pair<int, int> max_task, cur_task;

	if (priority_num == 0) priority_num = assign_list.size();	// ���û������������ȫ��Ϊ��ѡ����
	while (priority_num--) {
		cur_task = assign_list.front();
		assign_list.pop_front();

		int srcId = cur_task.first, tarId = cur_task.second;
		// �����˵�Դ����̨�ļ�ֵ
		double dist;
		if (this->work_id == srcId)	// ��������Դ����̨����
			dist = 2 * MAX_FORWARD_VEL * (time_per_frame * works[srcId]->remain_frames);	// 2vt
		else dist = calc_dist(this->x, this->y, works[srcId]->x, works[srcId]->y);		// d
		cur_value = value_coef * dist;
		cur_value += works[srcId]->value + works[tarId]->value;		// ����̨�ϵ���Ʒ��ֵ
		cur_value += value_w2w[srcId][tarId];		// Դ����̨��Ŀ�깤��̨�ļ�ֵ

		if (cur_value > max_value) {
			max_value = cur_value;
			max_task = cur_task;
		}
	}
	if (max_value > min_value) {		// �ɹ���������
		this->assignment = max_task;
		return true;
	}

	return false;	
}

Robot::~Robot() {}
