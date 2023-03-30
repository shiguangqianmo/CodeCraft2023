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
vector<vector<double>> value_w2w(MAX_K, vector<double>(MAX_K, 0));	// 50 * 50�ļ�ֵ��
vector<pair<int, int>> task_pair;	 // ��������� source work id -> target work id
deque<pair<int, int>> assign_list;	 // �����嵥��ǰ�����������񣬺����Ǳ�ѡ����
vector<int> resources(10, 0);		 // ȫ��ͼ��Դ

// ÿ�ֹ���̨�Ĺ����[type-1][0]���۳���[type-1][1]
int value_table[7][2] = { { 3000, 6000 },{ 4400, 7600 },{ 5800, 9200 },{ 15400, 22500 },{ 17200, 25000 },{ 19200, 27500 },{ 76000, 105000 } };

// ����̨�չ���Ʒ�������ݻ����������е�productӳ�䵽��Ӧ����̨
unordered_map<int, vector<int>> next_ws = { { 1,{ 4, 5, 9 } },{ 2,{ 4, 6, 9 } },{ 3,{ 5, 6, 9 } },{ 4,{ 7, 9 } },{ 5,{ 7, 9 } },{ 6,{ 7, 9 } },{ 7,{ 8, 9 } } };

// ָ��ӳ���
unordered_map<string, int> manipulation_dict{ { "forward", MANIPULATION::forward },
{ "rotate", MANIPULATION::rotate },
{ "buy", MANIPULATION::buy },
{ "sell", MANIPULATION::sell },
{ "destroy", MANIPULATION::destroy } };

bool init();        // ��ȡ��ͼ��Ϣ����ʼ������̨�ͻ�����
int get_task_pair_and_value();	// �������п��ܵ������ �Լ���� K*K ��ֵ��
bool read_frame();  // ��֡����
bool is_legal(pair<int, int>& tp);	// ����ԺϷ����ж�
int get_assign_list(vector<pair<int, int>>& task_pair);	// ���������嵥
int if_preemp(int robotId);		// �Ƿ������ռ ���� �ð汾δʹ��


int main() {
	//Sleep(15000);
	init();          // ��ʼ����ͼ��Ϣ
	get_task_pair_and_value(); // �������п��ܵ������
	puts("OK");      // ���߲��л���ʼ�����
	fflush(stdout);  // ������������

	// ��ʼ��֡����
	while (scanf("%d", &frameID) != EOF) {  // �������⣬����֡ID
		read_frame();                       // ��֡��Ϣ�������¹���̨�������˺��г���״̬
		if (frameID == 1386)
			cerr << "target frame:" << frameID << endl;


		for (int robotId = 0; robotId < 4; robotId++) {
			if (robots[robotId]->bind == -1) {	// ������δ�󶨹���̨��˵�������˿��У���Ϊ���������
				// ���������嵥����ȡ������������
				int priority_num = get_assign_list(task_pair);
				if (robots[robotId]->GetAssign(priority_num)) {		// �û����˳ɹ�������һ������
					// ���û����˰�Դ����̨
					robots[robotId]->bind = robots[robotId]->assignment.first;
				}
				else {		// ���û�������ܷ��� �ø�ʲô�أ�����
					continue;	// ��ʱ����	*** TODO ***
				}
			}
		}

		
		vector<pair<string, pair<int, float>>> manipulations;  // orders set
		// ��������£���ʱ��ÿ�������˶�����һ������̨����ΪԴ����̨����ΪĿ�깤��̨
		for (int robotId = 0; robotId < 4; robotId++) {
			if (robots[robotId]->bind != -1 && robots[robotId]->bind == robots[robotId]->work_id) {	// �����˾��ڰ󶨵Ĺ���̨����
				// �ж���Դ����̨����Ŀ�깤��̨
				if (robots[robotId]->bind == robots[robotId]->assignment.first) {	// ΪԴ����̨
					if (works[robots[robotId]->work_id]->product) {		// ȷ��Դ����̨���в�Ʒ
						manipulations.emplace_back(make_pair("buy", make_pair(robotId, 0.0)));
						robots[robotId]->product = works[robots[robotId]->work_id]->type;	// �����˳ɹ�ȡ��ԭ��
						robots[robotId]->bind = robots[robotId]->assignment.second;	// �����˰�Ŀ��̨
					}
				}
				else if (robots[robotId]->bind == robots[robotId]->assignment.second) {	// ΪĿ�깤��̨
					// ȷ��������Я������Ʒ����Ŀ�깤��̨�ܽ���
					if (robots[robotId]->product && works[robots[robotId]->work_id]->isUsed(robots[robotId]->product)) {
						manipulations.emplace_back(make_pair("sell", make_pair(robotId, 0.0)));
						robots[robotId]->product = 0;	// �����˳ɹ�������Ʒ
						robots[robotId]->bind = -1;	// �����˿���
						robots[robotId]->assignment = { -1, -1 };	// ��������
					}
				}// Ӧ�ò������else
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
				if (manipulation_dict[pp.first] == 1) {
					cerr << "vel_target" << pp.second.second << endl;
					cerr << "vel_cur" << robots[0]->v_x << " " << robots[0]->v_y << endl;
				}
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


// �������п��ܵ������ & ������ӦK*K�ļ�ֵ��
int get_task_pair_and_value() {
	for (int srcId = 0; srcId < K; srcId++)
		for (int tarId = 0; tarId < K; tarId++) {
			vector<int> next_set = next_ws[works[srcId]->type];
			if (find(next_set.begin(), next_set.end(), works[tarId]->type) != next_set.end()) {	// ���Ϲ���
				//if (works[tarId]->type == 9 && works[srcId]->type != 7)		// ����9ֻ����7
				//	continue;
				//else task_pair.emplace_back(make_pair(srcId, tarId));	// ����ȫ�������嵥
				task_pair.emplace_back(make_pair(srcId, tarId));
				
				// ��ֵ�������
				//0.12 = 6(����ٶ�) * 0.02(һ֡������)
				double p_frame = calc_dist(works[srcId]->x, works[srcId]->y, works[tarId]->x, works[tarId]->y) / 0.12;   
				double time_ceof = calc_time_coef(p_frame, 9000);  //����ʱ��ϵ��
				//���� = ����Ʒ�ۼ� * ʱ���ֵϵ�� * ��ײʱ��ϵ����- ��Ʒ����۸�
				value_w2w[srcId][tarId] = value_table[works[srcId]->type - 1][1] * time_ceof - value_table[works[srcId]->type - 1][0];
			}
		}
	return task_pair.size();
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
	int work_id;
	double t_c, c_c, o, v_x, v_y, d;
	for (int i = 0; i < 4; i++) {
		scanf("%d %d %lf %lf %lf %lf %lf %lf %lf %lf", &work_id, &product, &t_c, &c_c, &o, &v_x, &v_y, &d, &x, &y);
		robots[i]->updata_param(work_id, product, t_c, c_c, o, v_x, v_y, d, x, y);
	}

	char ok[5];
	scanf("%s", ok);
	if (ok[0] == 'O' && ok[1] == 'K') return true;

	return false;
}


// ����ԺϷ����ж�
// 1. ԭ��̨�в�Ʒ��������Ǳ�ڲ�Ʒ��������Ŀ��̨���Խ��ո�ԭ��
// 2. ��������������ȥͬһ��ԭ��̨������������������ͬһĿ��̨��һ���Ļ���������Ŀ��̨ͬʱΪ8��9��
// 3. ��������˽����������˵�Ŀ�깤��̨����Ϊ�Լ���Դ����̨��3.22������
bool is_legal(pair<int, int>& tp) {
	int srcId = tp.first, tarId = tp.second;
	for (int robotId = 0; robotId < 4; robotId++) {
		if (robots[robotId]->assignment.first == srcId ||		// ԭ��̨�Ѿ���һ̨������ռ��
			robots[robotId]->assignment.second == srcId)		// ����������ռ���˸�Ŀ�깤��̨
			return false;
		if (robots[robotId]->assignment.second == tarId && works[tarId]->type < 8 &&	// Ŀ��̨��ͬ�����Ҳ���8��9
			works[robots[robotId]->assignment.first]->type == works[srcId]->type)	// �����͵Ļ���ͬһ��
			return false;
	}
	return (works[srcId]->product || works[srcId]->remain_frames > 0) && works[tarId]->isUsed(works[srcId]->type);
}


// ���ɿɿ��������嵥��ǰpriority_num��Ϊ�������񣬺���Ϊ��ѡ����
// ����task_pairΪ���п��ܵ������
// ��assign_list���˫�˶��н��в��������������������priority_num
int get_assign_list(vector<pair<int, int>>& task_pair) {
	assign_list.clear();	// ��������嵥
	int priority_num = 0;	// ��������ĸ���
	for (auto cur_pair : task_pair) {
		if (!is_legal(cur_pair))	// ��������񲻺Ϸ�������
			continue;
		// �Ϸ��ż����嵥
		int tarId = cur_pair.second;
		// ���Ŀ�깤��̨Ϊ8/9��������4-7���ҳ���û�и���Դ���������Խ����ȿ���
		// if (works[tarId]->type >= 8 || (works[tarId]->type < 8 && resources[works[tarId]->type] == 0)) {
		if(resources[works[tarId]->type] == 0) {
			assign_list.push_front(cur_pair);
			priority_num++;
		}
		else assign_list.push_back(cur_pair);	// �����Ǳ�ѡ����
	}
	return priority_num;
}


// �ж��Ƿ������ռ
// robotId��������ռ�Ļ�����ID
// ���ؿ��Ա���ռ�Ļ����˵�id�����ܱ���ռ����-1
int if_preemp(int robotId) {
	for (int i = 0; i < 4; i++) {
		// ��ռ������������������Ĺ���̨������һ�������˵�ԭ��̨��������һ�������˻�û���
		if (i != robotId && robots[robotId]->work_id == robots[i]->assignment.first && robots[i]->product == 0)
			return i;
	}
	return -1;
}
