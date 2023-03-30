#include <iostream>
#include <cmath>
#include <algorithm>

#include "base.h"
using namespace std;

// ��������֮��ľ���
double calc_dist(double x1, double y1, double x2, double y2) {
	return pow(pow(x1 - x2, 2) + pow(y1 - y2, 2), 0.5);
}

//��ʽ����ʱ���ֵϵ������ײ��ֵϵ��
double calc_time_coef(double pocess_frame, int max_lost) {
	return (1 - sqrt(1 - pow((1 - (double)pocess_frame / max_lost), 2))) * 0.2 + 0.8;
}
