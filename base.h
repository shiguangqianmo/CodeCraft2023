#pragma once
#include <unordered_map>
#include <vector>
using namespace std;

constexpr double time_per_frame = 0.02;         // ÿ֡��ʱ��
constexpr int frequency = 50;                   // ֡��
constexpr double MAX_FORWARD_VEL = 6.0;
constexpr int MAX_K = 50;

// ÿ�ֹ���̨�Ĺ����[type-1][0]���۳���[type-1][1]
extern int value_table[7][2];
// ����̨�չ���Ʒ�������ݻ����������е�productӳ�䵽��Ӧ����̨
extern unordered_map<int, vector<int>> next_ws;
// ö���࣬��ʾ����
enum MANIPULATION { forward = 1, rotate, buy, sell, destroy };
extern unordered_map<string, int> manipulation_dict;

// ��������֮��ľ���
double calc_dist(double x1, double y1, double x2, double y2);

//����ʱ���ֵϵ��
double calc_time_coef(double pocess_frame, int max_lost);