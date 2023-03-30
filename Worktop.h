#pragma once
#include <iostream>
#include <vector>
#include <algorithm>
using namespace std;

class Worktop {
public:
	Worktop(int t, double x, double y);
	bool updata_param(int t, double x, double y, int remain, int r, int p);
	~Worktop();
	bool isUsed(int);
	double calc_value(int raw);		// ���㹤��̨��������Ʒ�ļ�ֵ

public:
	int type;           // ����̨���ͣ�int, [1, 9]��
	double x, y;        // ����
	int remain_frames;  // ʣ������֡����-1��û������0��������>=0��ʣ������֡����
	int raw;            // ԭ���ϸ�״̬
	int product;        // ��Ʒ��״̬��0��1��
	double value;			// �ù���̨��������Ʒ��ֵ
};

constexpr double value_8 = 0;		
constexpr double value_9 = 0;		// 8��9�Ź���̨�ļ�ֵ������
extern int value_table[7][2];
