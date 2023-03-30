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
	int work_id;                  // ��������̨ID��[-1������̨����-1]
	int product;                  // Я����Ʒ���ͣ�0��δЯ��
	double time_coef, coll_coef;  // ʱ�䡢��ײ��ֵϵ��[0.9, 1]
	double omega;                 // ���ٶȣ�>0����ʱ�룬<0��˳ʱ��
	double v_x, v_y;              // ���ٶ�
	double direction;             // ����[-pi, pi]
	double x, y;                  // ����
	int bind;                     // �󶨹���̨��ID��-1��ʾδ���κι���̨
	vector<int> assignment;		  // ������ eg:{1, 4, 7, 8}
};

constexpr double value_coef = -500;	// �����˵�Դ����̨�ļ�ֵϵ��
constexpr double min_value = value_coef * 100;	// ��С��ֵ�����벻�ᳬ��100��
extern vector<Worktop*> works;		// ���ɸ�����̨
extern vector<vector<double>> time_table;	// 50 * 50��ʱ���
extern vector<vector<double>> value_w2w;	// K * K�ļ�ֵ��
//extern deque<pair<int, int>> assign_list;	// �����嵥
extern vector<int> resources;			// ������Դ
extern vector<pair<int, int>> used_task_pair;	// ��ռ�õ�����ԣ���֡����ʱ����
