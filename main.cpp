#include <iostream>
#include <vector>
#include <queue>
#include <algorithm>
#include <Windows.h>

#include "Robot.h"
#include "Worktop.h"
#include "base.h"
using namespace std;

// 一些全局变量
int frameID = 0;         // 当前帧
int money;               // 当前金钱
int K;                   // 场上工作台的数量
vector<Worktop*> works;  // 若干个工作台
vector<Robot*> robots;   // 4个机器人指针
vector<vector<double>> value_w2w(MAX_K, vector<double>(MAX_K, 0));	// 50 * 50的价值表
vector<pair<int, int>> task_pair;	 // 所有任务对 source work id -> target work id
deque<pair<int, int>> assign_list;	 // 任务清单，前面是优先任务，后面是备选任务
vector<int> resources(10, 0);		 // 全地图资源

// 每种工作台的购买价[type-1][0]和售出价[type-1][1]
int value_table[7][2] = { { 3000, 6000 },{ 4400, 7600 },{ 5800, 9200 },{ 15400, 22500 },{ 17200, 25000 },{ 19200, 27500 },{ 76000, 105000 } };

// 工作台收购产品表，即根据机器人所持有的product映射到对应工作台
unordered_map<int, vector<int>> next_ws = { { 1,{ 4, 5, 9 } },{ 2,{ 4, 6, 9 } },{ 3,{ 5, 6, 9 } },{ 4,{ 7, 9 } },{ 5,{ 7, 9 } },{ 6,{ 7, 9 } },{ 7,{ 8, 9 } } };

// 指令映射表
unordered_map<string, int> manipulation_dict{ { "forward", MANIPULATION::forward },
{ "rotate", MANIPULATION::rotate },
{ "buy", MANIPULATION::buy },
{ "sell", MANIPULATION::sell },
{ "destroy", MANIPULATION::destroy } };

bool init();        // 读取地图信息，初始化工作台和机器人
int get_task_pair_and_value();	// 生成所有可能的任务对 以及获得 K*K 价值表
bool read_frame();  // 读帧数据
bool is_legal(pair<int, int>& tp);	// 任务对合法性判断
int get_assign_list(vector<pair<int, int>>& task_pair);	// 生成任务清单
int if_preemp(int robotId);		// 是否可以抢占 —— 该版本未使用


int main() {
	//Sleep(15000);
	init();          // 初始化地图信息
	get_task_pair_and_value(); // 生成所有可能的任务对
	puts("OK");      // 告诉裁判机初始化完成
	fflush(stdout);  // 清除输出缓冲区

	// 开始逐帧交互
	while (scanf("%d", &frameID) != EOF) {  // 还在判题，读到帧ID
		read_frame();                       // 读帧信息，并更新工作台、机器人和市场的状态
		if (frameID == 1386)
			cerr << "target frame:" << frameID << endl;


		for (int robotId = 0; robotId < 4; robotId++) {
			if (robots[robotId]->bind == -1) {	// 机器人未绑定工作台，说明机器人空闲，得为其分配任务
				// 更新任务清单，获取优先任务数量
				int priority_num = get_assign_list(task_pair);
				if (robots[robotId]->GetAssign(priority_num)) {		// 该机器人成功分配了一个任务
					// 将该机器人绑定源工作台
					robots[robotId]->bind = robots[robotId]->assignment.first;
				}
				else {		// 如果没有任务能分配 该干什么呢？？？
					continue;	// 暂时跳过	*** TODO ***
				}
			}
		}

		
		vector<pair<string, pair<int, float>>> manipulations;  // orders set
		// 正常情况下，此时，每个机器人都绑定了一个工作台，或为源工作台，或为目标工作台
		for (int robotId = 0; robotId < 4; robotId++) {
			if (robots[robotId]->bind != -1 && robots[robotId]->bind == robots[robotId]->work_id) {	// 机器人就在绑定的工作台附近
				// 判断是源工作台还是目标工作台
				if (robots[robotId]->bind == robots[robotId]->assignment.first) {	// 为源工作台
					if (works[robots[robotId]->work_id]->product) {		// 确定源工作台上有产品
						manipulations.emplace_back(make_pair("buy", make_pair(robotId, 0.0)));
						robots[robotId]->product = works[robots[robotId]->work_id]->type;	// 机器人成功取走原料
						robots[robotId]->bind = robots[robotId]->assignment.second;	// 机器人绑定目标台
					}
				}
				else if (robots[robotId]->bind == robots[robotId]->assignment.second) {	// 为目标工作台
					// 确保机器人携带了物品，且目标工作台能接收
					if (robots[robotId]->product && works[robots[robotId]->work_id]->isUsed(robots[robotId]->product)) {
						manipulations.emplace_back(make_pair("sell", make_pair(robotId, 0.0)));
						robots[robotId]->product = 0;	// 机器人成功卖掉产品
						robots[robotId]->bind = -1;	// 机器人空闲
						robots[robotId]->assignment = { -1, -1 };	// 清除任务对
					}
				}// 应该不会存在else
			}
			
			// 对于有绑定的机器人规划路径
			if (robots[robotId]->bind != -1) {
				int state = robots[robotId]->CheckCollide(robots, robotId);
				robots[robotId]->RotateAndVelControl(manipulations, works, robots, robotId, state);
			}
		}

		// 输出控制指令给判题器
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
		printf("OK\n");  // 指令发送完成，需要发送“OK”标志
		fflush(stdout);
	}
	return 0;
}


// 读取地图信息，初始化工作台和机器人
bool init() {
	char line[105];
	double cur_y = 49.75, cur_x;
	while (fgets(line, sizeof line, stdin)) {
		if (line[0] == 'O' && line[1] == 'K') {
			K = works.size();  // 工作台数目
			return true;       // 地图信息读完
		}
		cur_x = 0.25;
		for (int i = 0; i < 100; i++) {
			if (isdigit(line[i])) {  // 判断该点是工作台（该字符为数字）
				Worktop* work_ptr = new Worktop(line[i] - '0', cur_x, cur_y);
				works.emplace_back(work_ptr);
			}
			else if (line[i] == 'A') {  // 判断该点是机器人
				Robot* robot_ptr = new Robot(cur_x, cur_y);
				robots.emplace_back(robot_ptr);
			}
			cur_x += 0.5;
		}
		cur_y -= 0.5;
	}

	return false;
}


// 生成所有可能的任务对 & 生成相应K*K的价值表
int get_task_pair_and_value() {
	for (int srcId = 0; srcId < K; srcId++)
		for (int tarId = 0; tarId < K; tarId++) {
			vector<int> next_set = next_ws[works[srcId]->type];
			if (find(next_set.begin(), next_set.end(), works[tarId]->type) != next_set.end()) {	// 符合工序
				//if (works[tarId]->type == 9 && works[srcId]->type != 7)		// 限制9只接收7
				//	continue;
				//else task_pair.emplace_back(make_pair(srcId, tarId));	// 加入全局任务清单
				task_pair.emplace_back(make_pair(srcId, tarId));
				
				// 价值计算表项
				//0.12 = 6(最大速度) * 0.02(一帧的秒数)
				double p_frame = calc_dist(works[srcId]->x, works[srcId]->y, works[tarId]->x, works[tarId]->y) / 0.12;   
				double time_ceof = calc_time_coef(p_frame, 9000);  //计算时间系数
				//利润 = （物品售价 * 时间价值系数 * 碰撞时间系数）- 物品买入价格
				value_w2w[srcId][tarId] = value_table[works[srcId]->type - 1][1] * time_ceof - value_table[works[srcId]->type - 1][0];
			}
		}
	return task_pair.size();
}


// 读帧数据
bool read_frame() {
	scanf("%d %d", &money, &K);

	// 场上资源清空
	resources = vector<int>(10, 0);

	// 更新K个工作台的状态
	int t, remain, raw, product;
	double x, y;
	for (int k = 0; k < K; k++) {
		scanf("%d %lf %lf %d %d %d", &t, &x, &y, &remain, &raw, &product);
		works[k]->updata_param(t, x, y, remain, raw, product);
		
		resources[t] += product;		// 场上资源记录
		//resources[t] += (remain >= 0 ? 1 : 0);
	}

	// 更新4个机器人的状态
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


// 任务对合法性判断
// 1. 原料台有产品（或者有潜在产品），并且目标台可以接收该原料
// 2. 避免两个机器人去同一个原料台，避免两个机器人往同一目标台送一样的货（但允许目标台同时为8或9）
// 3. 避免机器人将其他机器人的目标工作台设置为自己的源工作台（3.22新增）
bool is_legal(pair<int, int>& tp) {
	int srcId = tp.first, tarId = tp.second;
	for (int robotId = 0; robotId < 4; robotId++) {
		if (robots[robotId]->assignment.first == srcId ||		// 原料台已经被一台机器人占用
			robots[robotId]->assignment.second == srcId)		// 其他机器人占用了该目标工作台
			return false;
		if (robots[robotId]->assignment.second == tarId && works[tarId]->type < 8 &&	// 目标台相同，并且不是8或9
			works[robots[robotId]->assignment.first]->type == works[srcId]->type)	// 并且送的货是同一种
			return false;
	}
	return (works[srcId]->product || works[srcId]->remain_frames > 0) && works[tarId]->isUsed(works[srcId]->type);
}


// 生成可靠的任务清单（前priority_num个为优先任务，后面为备选任务）
// 输入task_pair为所有可能的任务对
// 对assign_list这个双端队列进行操作，返回优先任务个数priority_num
int get_assign_list(vector<pair<int, int>>& task_pair) {
	assign_list.clear();	// 清空任务清单
	int priority_num = 0;	// 优先任务的个数
	for (auto cur_pair : task_pair) {
		if (!is_legal(cur_pair))	// 如果该任务不合法，跳过
			continue;
		// 合法才加入清单
		int tarId = cur_pair.second;
		// 如果目标工作台为8/9，或者是4-7并且场上没有该资源，这个任务对将优先考虑
		// if (works[tarId]->type >= 8 || (works[tarId]->type < 8 && resources[works[tarId]->type] == 0)) {
		if(resources[works[tarId]->type] == 0) {
			assign_list.push_front(cur_pair);
			priority_num++;
		}
		else assign_list.push_back(cur_pair);	// 否则是备选任务
	}
	return priority_num;
}


// 判断是否可以抢占
// robotId：发起抢占的机器人ID
// 返回可以被抢占的机器人的id，不能被抢占返回-1
int if_preemp(int robotId) {
	for (int i = 0; i < 4; i++) {
		// 抢占条件：机器人送完货的工作台，是另一个机器人的原料台，并且另一个机器人还没提货
		if (i != robotId && robots[robotId]->work_id == robots[i]->assignment.first && robots[i]->product == 0)
			return i;
	}
	return -1;
}
