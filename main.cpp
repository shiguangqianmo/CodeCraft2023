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
vector<vector<double>> time_table(MAX_K, vector<double>(MAX_K, 20));		// 50 * 50的时间表
vector<vector<double>> value_w2w(MAX_K, vector<double>(MAX_K, 0));	// 50 * 50的价值表
vector<vector<int>> task_link;		// 所有任务链
vector<int> resources(10, 0);		 // 全地图资源
vector<pair<int, int>> used_task_pair;	// 被占用的任务对，读帧数据时更新

// 每种工作台的购买价[type-1][0]和售出价[type-1][1]
int value_table[7][2] = { { 3000, 6000 },{ 4400, 7600 },{ 5800, 9200 },{ 15400, 22500 },{ 17200, 25000 },{ 19200, 27500 },{ 76000, 105000 } };

// 工作台收购产品表，即根据机器人所持有的product映射到对应工作台
unordered_map<int, vector<int>> next_ws = { { 1,{ 4, 5, 9 } },{ 2,{ 4, 6, 9 } },{ 3,{ 5, 6, 9 } },{ 4,{ 7, 9 } },{ 5,{ 7, 9 } },{ 6,{ 7, 9 } },{ 7,{ 8, 9 } } };
unordered_map<int, vector<int>> raw_list = { {4, {1, 2}}, {5, {1, 3}}, {6, {2, 3}},
											{7, {4, 5, 6}}, {8, {7}}, {9, {1, 2, 3, 4, 5, 6, 7}} };

// 指令映射表
unordered_map<string, int> manipulation_dict{ { "forward", MANIPULATION::forward },
{ "rotate", MANIPULATION::rotate },
{ "buy", MANIPULATION::buy },
{ "sell", MANIPULATION::sell },
{ "destroy", MANIPULATION::destroy } };


bool init();        // 读取地图信息，初始化工作台和机器人
void dfs(vector<int> temp_link);			// 递归建立任务链
void generate_link(vector<int>& head);		// 从任务链顶开始创建任务链
int get_task_link();						// 生成所有的任务链
void get_time_and_value();	// 生成所有可能的任务对 以及获得 K*K 价值表
bool read_frame();  // 读帧数据


int main() {
	//Sleep(15000);
	init();          // 初始化地图信息
	get_time_and_value();
	get_task_link();	// 从任务链顶开始创建任务链

	for (int robotId = 0; robotId < 4; robotId++) {
		// 给四个机器人分配初始任务
		if (robots[robotId]->GetAssign(task_link)) {	// 成功分配一个任务链
			robots[robotId]->bind = robots[robotId]->assignment[0];
		}// 应该不存在else
	}

	puts("OK");      // 告诉裁判机初始化完成
	fflush(stdout);  // 清除输出缓冲区

	// 开始逐帧交互
	while (scanf("%d", &frameID) != EOF) {  // 还在判题，读到帧ID
		read_frame();                       // 读帧信息，并更新工作台、机器人和市场的状态
		if (frameID == 1734)
			cerr << "target frame:" << frameID << endl;

		// 正常情况下，机器人一定是有任务在手的
		vector<pair<string, pair<int, float>>> manipulations;  // orders set
		// 此时，每个机器人都绑定了一个工作台，可能是：源工作台 or （中间工作台 or）终结工作台
		for (int robotId = 0; robotId < 4; robotId++) {
			if (robots[robotId]->bind == -1) {	// 机器人没有任务
				if (robots[robotId]->GetAssign(task_link)) {
					robots[robotId]->bind = robots[robotId]->assignment[0];
				}
			}

			if (robots[robotId]->bind != -1 && robots[robotId]->bind == robots[robotId]->work_id) {	// 机器人就在绑定的工作台附近
				int bind_idx = 0;
				while (robots[robotId]->bind != robots[robotId]->assignment[bind_idx]) bind_idx++;
				// 判断绑定的是什么工作台
				if (bind_idx == 0) {	// 为源工作台
					if (works[robots[robotId]->work_id]->product) {	// 确定源工作台上有产品
						manipulations.emplace_back(make_pair("buy", make_pair(robotId, 0.0)));
						robots[robotId]->product = works[robots[robotId]->work_id]->type;	// 机器人成功取走原料
						robots[robotId]->bind = robots[robotId]->assignment[bind_idx + 1];	// 绑定任务链的下一个工作台
					}
				}
				else if (bind_idx == robots[robotId]->assignment.size() - 1) {	// 终结工作台
					// 确保机器人携带了物品，且目标工作台能接收
					if (robots[robotId]->product && works[robots[robotId]->work_id]->isUsed(robots[robotId]->product)) {
						manipulations.emplace_back(make_pair("sell", make_pair(robotId, 0.0)));
						// 为该机器人分配新的任务
						if (robots[robotId]->GetAssign(task_link)) {
							robots[robotId]->bind = robots[robotId]->assignment[0];
						}
						else {	// 没有成功分配的情况
							robots[robotId]->assignment = {};
							robots[robotId]->bind = -1;
						}
					}
				}
				else {	// 为中间工作台，机器人能先卖原料再买中间产品
					// 确保机器人携带了物品，且目标工作台能接收
					if (robots[robotId]->product && works[robots[robotId]->work_id]->isUsed(robots[robotId]->product)) {
						manipulations.emplace_back(make_pair("sell", make_pair(robotId, 0.0)));
					}
					// 确定中间工作台上有产品
					if (works[robots[robotId]->work_id]->product) {
						manipulations.emplace_back(make_pair("buy", make_pair(robotId, 0.0)));
						robots[robotId]->product = works[robots[robotId]->work_id]->type;	// 机器人成功取走原料
						robots[robotId]->bind = robots[robotId]->assignment[bind_idx + 1];	// 绑定任务链的下一个工作台
					}
				}
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


// 生成相应K*K的时间表 & 价值表
void get_time_and_value() {
	for (int srcId = 0; srcId < K; srcId++)
		for (int tarId = 0; tarId < K; tarId++) {
			vector<int> next_set = next_ws[works[srcId]->type];
			if (find(next_set.begin(), next_set.end(), works[tarId]->type) != next_set.end()) {	// 符合工序			
				// 时间、价值计算表项
				double dist = calc_dist(works[srcId]->x, works[srcId]->y, works[tarId]->x, works[tarId]->y);
				double t = dist / MAX_FORWARD_VEL;		// srcId到tarId所需要的时间
				double time_ceof = calc_time_coef(t / time_per_frame, 9000);  //计算时间系数
				//利润 = （物品售价 * 时间价值系数 * 碰撞时间系数）- 物品买入价格
				time_table[srcId][tarId] = t;
				value_w2w[srcId][tarId] = value_table[works[srcId]->type - 1][1] * time_ceof - value_table[works[srcId]->type - 1][0];
			}
		}
}


// 生成所有的任务链
int get_task_link() {
	unordered_map<int, vector<int>> work_set;
	for (int workId = 0; workId < K; workId++) {
		work_set[works[workId]->type].emplace_back(workId);		// 将工作台分类
	}

	if (work_set[9].size() > 0) {	// 有9号工作台
		generate_link(work_set[9]);
	}
	else if (work_set[8].size() == 0 && work_set[7].size() > 0) {	// 没有9号，没有8号，有7号
		generate_link(work_set[7]);
	}
	else if (work_set[8].size() > 0 && work_set[7].size() > 0) {	// 没有9号，有8有7
		generate_link(work_set[8]);
	}
	else {	// 7、8、9都没有，或者有8没有7、9
		generate_link(work_set[4]);
		generate_link(work_set[5]);
		generate_link(work_set[6]);
	}
	return task_link.size();
}


// 从任务链顶开始创建任务链
void generate_link(vector<int>& head) {
	for (auto cur_id : head) {
		vector<int> init_link = { cur_id };
		dfs(init_link);
	}
}


// 递归建立任务链
void dfs(vector<int> temp_link) {
	int cur_type = works[temp_link[temp_link.size() - 1]]->type;
	vector<int> raws = raw_list[cur_type];
	for (int workId = 0; workId < K; workId++) {
		// workId工作台生产的产品是原料
		if (find(raws.begin(), raws.end(), works[workId]->type) != raws.end()) {
			vector<int> add_link = temp_link;
			add_link.emplace_back(workId);
			if (works[workId]->type < 4) {	// 1/2/3，任务结束
				reverse(add_link.begin(), add_link.end());
				task_link.emplace_back(add_link);	// 加入任务链
			}
			else {
				dfs(add_link);
			}
		}
	}
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
	used_task_pair = {};	// 清空任务对，重新记录
	int work_id;
	double t_c, c_c, o, v_x, v_y, d;
	for (int robotId = 0; robotId < 4; robotId++) {
		scanf("%d %d %lf %lf %lf %lf %lf %lf %lf %lf", &work_id, &product, &t_c, &c_c, &o, &v_x, &v_y, &d, &x, &y);
		robots[robotId]->updata_param(work_id, product, t_c, c_c, o, v_x, v_y, d, x, y);

		// 将机器人的任务链以任务对的形式存入全局占用任务中
		vector<int> cur_link = robots[robotId]->assignment;	// 当前机器人的任务链
		if (!cur_link.empty()) {
			int bind_idx = 0;
			while (robots[robotId]->bind != cur_link[bind_idx]) bind_idx++;
			if (bind_idx == 0) {	// 如果机器人绑定的是任务链的第一个工作台
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


// 任务对合法性判断
// 1. （原料台上有产品 or 机器人到原料台时能生产出来），and 目标台接收该原料
// 2. 避免两个机器人去同一个原料台
// 3. 避免两个机器人往同一目标台送一样的货（但允许目标台同时为8或9）
//bool is_legal(double time, pair<int, int>& tp) {
//	int srcId = tp.first, tarId = tp.second;
//	for (auto cmp_pair : used_task_pair) {
//		if (cmp_pair.first == srcId)	// 条件2：原料台已经被一台机器人占用
//			return false;
//		if (cmp_pair.second == tarId && works[tarId]->type < 8 &&	// 条件3：目标台相同，并且不是8或9
//			works[cmp_pair.first]->type && works[srcId]->type)	// 并且送的货是同一种
//			return false;
//	}
//	// 条件1
//	return (works[srcId]->product || time >= works[srcId]->remain_frames * time_per_frame) && works[tarId]->isUsed(works[srcId]->type);
//}


// 生成可靠的任务清单（前priority_num个为优先任务，后面为备选任务）
// 输入task_pair为所有可能的任务对
// 对assign_list这个双端队列进行操作，返回优先任务个数priority_num
//int get_assign_list(vector<pair<int, int>>& task_pair) {
//	assign_list.clear();	// 清空任务清单
//	int priority_num = 0;	// 优先任务的个数
//	for (auto cur_pair : task_pair) {
//		if (!is_legal(cur_pair))	// 如果该任务不合法，跳过
//			continue;
//		// 合法才加入清单
//		int tarId = cur_pair.second;
//		// 如果目标工作台为8/9，或者是4-7并且场上没有该资源，这个任务对将优先考虑
//		// if (works[tarId]->type >= 8 || (works[tarId]->type < 8 && resources[works[tarId]->type] == 0)) {
//		if(resources[works[tarId]->type] == 0) {
//			assign_list.push_front(cur_pair);
//			priority_num++;
//		}
//		else assign_list.push_back(cur_pair);	// 否则是备选任务
//	}
//	return priority_num;
//}

