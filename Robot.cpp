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
	assignment = {};			// ��ʼ����û������
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


// ����������ѡ��һ���Ϸ��ġ���ֵ��ߵ�����
bool Robot::GetAssign(vector<vector<int>>& task_link) {
	using PAIR = pair<vector<int>, int>;
	auto cmp = [&](PAIR& a, PAIR& b) { return a.second < b.second; };
	priority_queue<PAIR, vector<PAIR>, decltype(cmp)> prior_assign(cmp);	// ���ȵ�������
	priority_queue<PAIR, vector<PAIR>, decltype(cmp)> alter_assign(cmp);	// ��ѡ��������
	for (auto& link : task_link) {
		vector<pair<int, int>> cur_ans;	// ��Ч������Դ���cur_ans
		int last_work_id = -1;		// ��һ������̨��id
		double cur_value = 0;
		double time = 0;
		for (int i = 0; i < link.size() - 1; i++) {
			if (i == 0 || last_work_id != link[i]) {	// �׸�����̨�����������׸�����̨
				time = calc_dist(x, y, works[link[i]]->x, works[link[i]]->y) / MAX_FORWARD_VEL;
			}
			else
				time += time_table[link[i]][link[i - 1]];

			pair<int, int> cur_task = make_pair(link[i], link[i + 1]);
			if (is_legal(time, cur_task)) {
				cur_ans.emplace_back(cur_task);
				last_work_id = cur_task.second;
			}
		}

		// ����Ч��vector pair��ת��Ϊvector int��������value���������ȶ���
		vector<int> cur_list;
		if (cur_ans.size() == 0)
			continue;
		else if (cur_ans.size() == 1) {		// 2�ڵ�����������̣�
			//if (cur_ans[0].first == 29 || cur_ans[0].first == 47)
			//	cerr << "target" << endl;
			double dist = calc_dist(this->x, this->y, works[cur_ans[0].first]->x, works[cur_ans[0].first]->y);
			cur_value = value_coef * dist;		// �����˵�Դ����̨�ľ�����ʧ
			cur_value += value_w2w[cur_ans[0].first][cur_ans[0].second];  // Դ����̨��Ŀ�깤��̨�ļ�ֵ
			cur_list.emplace_back(cur_ans[0].first);
			cur_list.emplace_back(cur_ans[0].second);
			// Ŀ��̨����9 && ����û�и�Ŀ���Ʒ
			if (works[cur_ans[0].second]->type != 9 && !resources[works[cur_ans[0].second]->type])
				prior_assign.emplace(make_pair(cur_list, cur_value));	// ��Ϊ��������
			else
				alter_assign.emplace(make_pair(cur_list, cur_value));	// ��Ϊ��ѡ����
		}
		else if (cur_ans.size() == 2) {		// 2������ԣ�2�����
			if (cur_ans[0].second == cur_ans[1].first) {	// �������������
				cur_list.emplace_back(cur_ans[0].first);
				cur_list.emplace_back(cur_ans[1].first);
				cur_list.emplace_back(cur_ans[1].second);
				double dist = calc_dist(this->x, this->y, works[cur_ans[0].first]->x, works[cur_ans[0].first]->y);
				cur_value = value_coef * dist;
				cur_value += value_w2w[cur_ans[0].first][cur_ans[0].second];
				cur_value += value_w2w[cur_ans[1].first][cur_ans[1].second];

				if(works[cur_ans[1].second]->type != 9)
					prior_assign.emplace(make_pair(cur_list, cur_value));	// ��Ϊ��������
				else
					alter_assign.emplace(make_pair(cur_list, cur_value));	// ��Ϊ��ѡ����
			}
			else {	// �����������������Ҫ��������������ļ�ֵ
				double value0 = value_coef * calc_dist(this->x, this->y, works[cur_ans[0].first]->x, works[cur_ans[0].first]->y);
				double value1 = value_coef * calc_dist(this->x, this->y, works[cur_ans[1].first]->x, works[cur_ans[1].first]->y);
				value0 += value_w2w[cur_ans[0].first][cur_ans[0].second];
				value1 += value_w2w[cur_ans[1].first][cur_ans[1].second];

				// ����1
				cur_list.emplace_back(cur_ans[0].first);
				cur_list.emplace_back(cur_ans[0].second);
				if(!resources[works[cur_ans[0].second]->type])
					prior_assign.emplace(make_pair(cur_list, cur_value));	// ��Ϊ��������
				else
					alter_assign.emplace(make_pair(cur_list, cur_value));	// ��Ϊ��ѡ����

				// ����2
				cur_list = {};
				cur_list.emplace_back(cur_ans[1].first);
				cur_list.emplace_back(cur_ans[1].second);
				prior_assign.emplace(make_pair(cur_list, cur_value));	// ��Ϊ��������
			}
		}
		else {	// 3������ԣ���4�ڵ������������
			cur_list.emplace_back(cur_ans[0].first);
			cur_list.emplace_back(cur_ans[1].first);
			cur_list.emplace_back(cur_ans[2].first);
			cur_list.emplace_back(cur_ans[2].second);
			double dist = calc_dist(this->x, this->y, works[cur_ans[0].first]->x, works[cur_ans[0].first]->y);
			cur_value = value_coef * dist;
			cur_value += value_w2w[cur_ans[0].first][cur_ans[0].second];
			cur_value += value_w2w[cur_ans[1].first][cur_ans[1].second];
			cur_value += value_w2w[cur_ans[2].first][cur_ans[2].second];
			prior_assign.emplace(make_pair(cur_list, cur_value));	// ��Ϊ��������
		}
	}
	// ����
	if (!prior_assign.empty()) {
		this->assignment = prior_assign.top().first;
		this->updata_used_task_pair(assignment);	// ����ռ�õ������
		return true;
	}
	if (!alter_assign.empty()) {
		this->assignment = alter_assign.top().first;
		this->updata_used_task_pair(assignment);	// ����ռ�õ������
		return true;
	}
	return false;
}


void Robot::updata_used_task_pair(vector<int>& assign) {
	for (int i = 0; i < assign.size() - 1; i++)
		used_task_pair.emplace_back(make_pair(assign[i], assign[i + 1]));
}



bool Robot::is_legal(double time, pair<int, int>& tp) {
	int srcId = tp.first, tarId = tp.second;
	for (auto cmp_pair : used_task_pair) {
		if (cmp_pair.first == srcId)	// ����2��ԭ��̨�Ѿ���һ̨������ռ��
			return false;
		if (cmp_pair.second == tarId && works[tarId]->type < 8 &&	// ����3��Ŀ��̨��ͬ�����Ҳ���8��9
			works[cmp_pair.first]->type == works[srcId]->type)	// �����͵Ļ���ͬһ��
			return false;
	}
	// ����1
	return (works[srcId]->product || ((works[srcId]->remain_frames > -1) && time >= works[srcId]->remain_frames * time_per_frame))
		&& works[tarId]->isUsed(works[srcId]->type);
}

Robot::~Robot() {}
