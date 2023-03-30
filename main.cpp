#include <iostream>
#include <vector>
#include <queue>
#include <algorithm>
#include <Windows.h>

#include "Robot.h"
#include "Worktop.h"
#include "base.h"
using namespace std;

// һЩȫ�ֱ���
int frameID = 0;         // ��ǰ֡
int money;               // ��ǰ��Ǯ
int K;                   // ���Ϲ���̨������
vector<Worktop*> works;  // ���ɸ�����̨
vector<Robot*> robots;   // 4��������ָ��
vector<vector<double>> time_table(MAX_K, vector<double>(MAX_K, 20));		// 50 * 50��ʱ���
vector<vector<double>> value_w2w(MAX_K, vector<double>(MAX_K, 0));	// 50 * 50�ļ�ֵ��
vector<vector<int>> task_link;		// ����������
vector<int> resources(10, 0);		 // ȫ��ͼ��Դ
vector<pair<int, int>> used_task_pair;	// ��ռ�õ�����ԣ���֡����ʱ����

// ÿ�ֹ���̨�Ĺ����[type-1][0]���۳���[type-1][1]
int value_table[7][2] = { { 3000, 6000 },{ 4400, 7600 },{ 5800, 9200 },{ 15400, 22500 },{ 17200, 25000 },{ 19200, 27500 },{ 76000, 105000 } };

// ����̨�չ���Ʒ�������ݻ����������е�productӳ�䵽��Ӧ����̨
unordered_map<int, vector<int>> next_ws = { { 1,{ 4, 5, 9 } },{ 2,{ 4, 6, 9 } },{ 3,{ 5, 6, 9 } },{ 4,{ 7, 9 } },{ 5,{ 7, 9 } },{ 6,{ 7, 9 } },{ 7,{ 8, 9 } } };
unordered_map<int, vector<int>> raw_list = { {4, {1, 2}}, {5, {1, 3}}, {6, {2, 3}},
											{7, {4, 5, 6}}, {8, {7}}, {9, {1, 2, 3, 4, 5, 6, 7}} };

// ָ��ӳ���
unordered_map<string, int> manipulation_dict{ { "forward", MANIPULATION::forward },
{ "rotate", MANIPULATION::rotate },
{ "buy", MANIPULATION::buy },
{ "sell", MANIPULATION::sell },
{ "destroy", MANIPULATION::destroy } };


bool init();        // ��ȡ��ͼ��Ϣ����ʼ������̨�ͻ�����
void dfs(vector<int> temp_link);			// �ݹ齨��������
void generate_link(vector<int>& head);		// ������������ʼ����������
int get_task_link();						// �������е�������
void get_time_and_value();	// �������п��ܵ������ �Լ���� K*K ��ֵ��
bool read_frame();  // ��֡����


int main() {
	//Sleep(15000);
	init();          // ��ʼ����ͼ��Ϣ
	get_time_and_value();
	get_task_link();	// ������������ʼ����������

	for (int robotId = 0; robotId < 4; robotId++) {
		// ���ĸ������˷����ʼ����
		if (robots[robotId]->GetAssign(task_link)) {	// �ɹ�����һ��������
			robots[robotId]->bind = robots[robotId]->assignment[0];
		}// Ӧ�ò�����else
	}

	puts("OK");      // ���߲��л���ʼ�����
	fflush(stdout);  // ������������

	// ��ʼ��֡����
	while (scanf("%d", &frameID) != EOF) {  // �������⣬����֡ID
		read_frame();                       // ��֡��Ϣ�������¹���̨�������˺��г���״̬
		if (frameID == 1734)
			cerr << "target frame:" << frameID << endl;

		// ��������£�������һ�������������ֵ�
		vector<pair<string, pair<int, float>>> manipulations;  // orders set
		// ��ʱ��ÿ�������˶�����һ������̨�������ǣ�Դ����̨ or ���м乤��̨ or���սṤ��̨
		for (int robotId = 0; robotId < 4; robotId++) {
			if (robots[robotId]->bind == -1) {	// ������û������
				if (robots[robotId]->GetAssign(task_link)) {
					robots[robotId]->bind = robots[robotId]->assignment[0];
				}
			}

			if (robots[robotId]->bind != -1 && robots[robotId]->bind == robots[robotId]->work_id) {	// �����˾��ڰ󶨵Ĺ���̨����
				int bind_idx = 0;
				while (robots[robotId]->bind != robots[robotId]->assignment[bind_idx]) bind_idx++;
				// �жϰ󶨵���ʲô����̨
				if (bind_idx == 0) {	// ΪԴ����̨
					if (works[robots[robotId]->work_id]->product) {	// ȷ��Դ����̨���в�Ʒ
						manipulations.emplace_back(make_pair("buy", make_pair(robotId, 0.0)));
						robots[robotId]->product = works[robots[robotId]->work_id]->type;	// �����˳ɹ�ȡ��ԭ��
						robots[robotId]->bind = robots[robotId]->assignment[bind_idx + 1];	// ������������һ������̨
					}
				}
				else if (bind_idx == robots[robotId]->assignment.size() - 1) {	// �սṤ��̨
					// ȷ��������Я������Ʒ����Ŀ�깤��̨�ܽ���
					if (robots[robotId]->product && works[robots[robotId]->work_id]->isUsed(robots[robotId]->product)) {
						manipulations.emplace_back(make_pair("sell", make_pair(robotId, 0.0)));
						// Ϊ�û����˷����µ�����
						if (robots[robotId]->GetAssign(task_link)) {
							robots[robotId]->bind = robots[robotId]->assignment[0];
						}
						else {	// û�гɹ���������
							robots[robotId]->assignment = {};
							robots[robotId]->bind = -1;
						}
					}
				}
				else {	// Ϊ�м乤��̨��������������ԭ�������м��Ʒ
					// ȷ��������Я������Ʒ����Ŀ�깤��̨�ܽ���
					if (robots[robotId]->product && works[robots[robotId]->work_id]->isUsed(robots[robotId]->product)) {
						manipulations.emplace_back(make_pair("sell", make_pair(robotId, 0.0)));
					}
					// ȷ���м乤��̨���в�Ʒ
					if (works[robots[robotId]->work_id]->product) {
						manipulations.emplace_back(make_pair("buy", make_pair(robotId, 0.0)));
						robots[robotId]->product = works[robots[robotId]->work_id]->type;	// �����˳ɹ�ȡ��ԭ��
						robots[robotId]->bind = robots[robotId]->assignment[bind_idx + 1];	// ������������һ������̨
					}
				}
			}
			
			// �����а󶨵Ļ����˹滮·��
			if (robots[robotId]->bind != -1) {
				int state = robots[robotId]->CheckCollide(robots, robotId);
				robots[robotId]->RotateAndVelControl(manipulations, works, robots, robotId, state);
			}
		}

		// �������ָ���������
		printf("%d\n", frameID);
		for (auto& pp : manipulations) {
			if (manipulation_dict[pp.first] <= 2) {
				printf("%s %d %lf\n", pp.first.c_str(), pp.second.first, pp.second.second);
			}
			else
				printf("%s %d\n", pp.first.c_str(), pp.second.first);
		}
		printf("OK\n");  // ָ�����ɣ���Ҫ���͡�OK����־
		fflush(stdout);
	}
	return 0;
}


// ��ȡ��ͼ��Ϣ����ʼ������̨�ͻ�����
bool init() {
	char line[105];
	double cur_y = 49.75, cur_x;
	while (fgets(line, sizeof line, stdin)) {
		if (line[0] == 'O' && line[1] == 'K') {
			K = works.size();  // ����̨��Ŀ
			return true;       // ��ͼ��Ϣ����
		}
		cur_x = 0.25;
		for (int i = 0; i < 100; i++) {
			if (isdigit(line[i])) {  // �жϸõ��ǹ���̨�����ַ�Ϊ���֣�
				Worktop* work_ptr = new Worktop(line[i] - '0', cur_x, cur_y);
				works.emplace_back(work_ptr);
			}
			else if (line[i] == 'A') {  // �жϸõ��ǻ�����
				Robot* robot_ptr = new Robot(cur_x, cur_y);
				robots.emplace_back(robot_ptr);
			}
			cur_x += 0.5;
		}
		cur_y -= 0.5;
	}

	return false;
}


// ������ӦK*K��ʱ��� & ��ֵ��
void get_time_and_value() {
	for (int srcId = 0; srcId < K; srcId++)
		for (int tarId = 0; tarId < K; tarId++) {
			vector<int> next_set = next_ws[works[srcId]->type];
			if (find(next_set.begin(), next_set.end(), works[tarId]->type) != next_set.end()) {	// ���Ϲ���			
				// ʱ�䡢��ֵ�������
				double dist = calc_dist(works[srcId]->x, works[srcId]->y, works[tarId]->x, works[tarId]->y);
				double t = dist / MAX_FORWARD_VEL;		// srcId��tarId����Ҫ��ʱ��
				double time_ceof = calc_time_coef(t / time_per_frame, 9000);  //����ʱ��ϵ��
				//���� = ����Ʒ�ۼ� * ʱ���ֵϵ�� * ��ײʱ��ϵ����- ��Ʒ����۸�
				time_table[srcId][tarId] = t;
				value_w2w[srcId][tarId] = value_table[works[srcId]->type - 1][1] * time_ceof - value_table[works[srcId]->type - 1][0];
			}
		}
}


// �������е�������
int get_task_link() {
	unordered_map<int, vector<int>> work_set;
	for (int workId = 0; workId < K; workId++) {
		work_set[works[workId]->type].emplace_back(workId);		// ������̨����
	}

	if (work_set[9].size() > 0) {	// ��9�Ź���̨
		generate_link(work_set[9]);
	}
	else if (work_set[8].size() == 0 && work_set[7].size() > 0) {	// û��9�ţ�û��8�ţ���7��
		generate_link(work_set[7]);
	}
	else if (work_set[8].size() > 0 && work_set[7].size() > 0) {	// û��9�ţ���8��7
		generate_link(work_set[8]);
	}
	else {	// 7��8��9��û�У�������8û��7��9
		generate_link(work_set[4]);
		generate_link(work_set[5]);
		generate_link(work_set[6]);
	}
	return task_link.size();
}


// ������������ʼ����������
void generate_link(vector<int>& head) {
	for (auto cur_id : head) {
		vector<int> init_link = { cur_id };
		dfs(init_link);
	}
}


// �ݹ齨��������
void dfs(vector<int> temp_link) {
	int cur_type = works[temp_link[temp_link.size() - 1]]->type;
	vector<int> raws = raw_list[cur_type];
	for (int workId = 0; workId < K; workId++) {
		// workId����̨�����Ĳ�Ʒ��ԭ��
		if (find(raws.begin(), raws.end(), works[workId]->type) != raws.end()) {
			vector<int> add_link = temp_link;
			add_link.emplace_back(workId);
			if (works[workId]->type < 4) {	// 1/2/3���������
				reverse(add_link.begin(), add_link.end());
				task_link.emplace_back(add_link);	// ����������
			}
			else {
				dfs(add_link);
			}
		}
	}
}


// ��֡����
bool read_frame() {
	scanf("%d %d", &money, &K);

	// ������Դ���
	resources = vector<int>(10, 0);

	// ����K������̨��״̬
	int t, remain, raw, product;
	double x, y;
	for (int k = 0; k < K; k++) {
		scanf("%d %lf %lf %d %d %d", &t, &x, &y, &remain, &raw, &product);
		works[k]->updata_param(t, x, y, remain, raw, product);
		
		resources[t] += product;		// ������Դ��¼
		//resources[t] += (remain >= 0 ? 1 : 0);
	}

	// ����4�������˵�״̬
	used_task_pair = {};	// �������ԣ����¼�¼
	int work_id;
	double t_c, c_c, o, v_x, v_y, d;
	for (int robotId = 0; robotId < 4; robotId++) {
		scanf("%d %d %lf %lf %lf %lf %lf %lf %lf %lf", &work_id, &product, &t_c, &c_c, &o, &v_x, &v_y, &d, &x, &y);
		robots[robotId]->updata_param(work_id, product, t_c, c_c, o, v_x, v_y, d, x, y);

		// �������˵�������������Ե���ʽ����ȫ��ռ��������
		vector<int> cur_link = robots[robotId]->assignment;	// ��ǰ�����˵�������
		if (!cur_link.empty()) {
			int bind_idx = 0;
			while (robots[robotId]->bind != cur_link[bind_idx]) bind_idx++;
			if (bind_idx == 0) {	// ��������˰󶨵����������ĵ�һ������̨
				for (int i = 0; i < cur_link.size() - 1; i++)
					used_task_pair.emplace_back(make_pair(cur_link[i], cur_link[i + 1]));
			}
			else {
				for (int i = bind_idx; i < cur_link.size(); i++)
					used_task_pair.emplace_back(make_pair(cur_link[i - 1], cur_link[i]));
			}
		}
	}

	char ok[5];
	scanf("%s", ok);
	if (ok[0] == 'O' && ok[1] == 'K') return true;

	return false;
}


// ����ԺϷ����ж�
// 1. ��ԭ��̨���в�Ʒ or �����˵�ԭ��̨ʱ��������������and Ŀ��̨���ո�ԭ��
// 2. ��������������ȥͬһ��ԭ��̨
// 3. ����������������ͬһĿ��̨��һ���Ļ���������Ŀ��̨ͬʱΪ8��9��
//bool is_legal(double time, pair<int, int>& tp) {
//	int srcId = tp.first, tarId = tp.second;
//	for (auto cmp_pair : used_task_pair) {
//		if (cmp_pair.first == srcId)	// ����2��ԭ��̨�Ѿ���һ̨������ռ��
//			return false;
//		if (cmp_pair.second == tarId && works[tarId]->type < 8 &&	// ����3��Ŀ��̨��ͬ�����Ҳ���8��9
//			works[cmp_pair.first]->type && works[srcId]->type)	// �����͵Ļ���ͬһ��
//			return false;
//	}
//	// ����1
//	return (works[srcId]->product || time >= works[srcId]->remain_frames * time_per_frame) && works[tarId]->isUsed(works[srcId]->type);
//}


// ���ɿɿ��������嵥��ǰpriority_num��Ϊ�������񣬺���Ϊ��ѡ����
// ����task_pairΪ���п��ܵ������
// ��assign_list���˫�˶��н��в��������������������priority_num
//int get_assign_list(vector<pair<int, int>>& task_pair) {
//	assign_list.clear();	// ��������嵥
//	int priority_num = 0;	// ��������ĸ���
//	for (auto cur_pair : task_pair) {
//		if (!is_legal(cur_pair))	// ��������񲻺Ϸ�������
//			continue;
//		// �Ϸ��ż����嵥
//		int tarId = cur_pair.second;
//		// ���Ŀ�깤��̨Ϊ8/9��������4-7���ҳ���û�и���Դ���������Խ����ȿ���
//		// if (works[tarId]->type >= 8 || (works[tarId]->type < 8 && resources[works[tarId]->type] == 0)) {
//		if(resources[works[tarId]->type] == 0) {
//			assign_list.push_front(cur_pair);
//			priority_num++;
//		}
//		else assign_list.push_back(cur_pair);	// �����Ǳ�ѡ����
//	}
//	return priority_num;
//}

