#include "Worktop.h"

// ���캯������ʼ��һ������̨״̬
Worktop::Worktop(int t, double point_x, double point_y) {
	type = t;
	x = point_x;
	y = point_y;
	remain_frames = (type < 4 ? 50 : -1);	// ��ʼ����ʱ���1��2��3����̨����ʣ��֡������������˷����ʼ����
	raw = 0;
	product = 0;
	value = 0;
}

// ���ݶ�ȡ��֡���ݽ���״̬����
bool Worktop::updata_param(int t, double point_x, double point_y, int remain, int r, int p) {
	type = t;
	x = point_x;
	y = point_y;
	remain_frames = remain;
	raw = r;
	product = p;
	value = this->calc_value(raw);
	return true;
}


// �ù���̨�����ࣨcarried_type��ԭ�ϸ��Ƿ��Ѿ���ռ��
// ��ռ���˷���false�����ŷ���true		******** �������Ƿ�Ҫ��һ��
bool Worktop::isUsed(int carried_type) {
	return ((1 << carried_type) & raw) == 0;
}


// ���㹤��̨��������Ʒ�ļ�ֵ�����󣩣�����Ǳ�ڲ�Ʒ
double Worktop::calc_value(int raw) {
	if (type == 8)
		return value_8;
	if (type == 9)
		return value_9;
	if (type == 7 && !product && remain_frames < 0)
		return 3400 * 2;	// ���7��̨û�в�Ʒ��Ǳ�ڲ�Ʒ������7һ����3��̨�൱�ļ�ֵ

	double ans = 0;
	if (product)	// ����в�Ʒ�����ϲ�Ʒ��ֵ������
		ans += value_table[type - 1][1] - value_table[type - 1][0];
	if(remain_frames > 0)	// �������������Ʒ���ټ�һ����Ʒ����	��3.21������
		ans += value_table[type - 1][1] - value_table[type - 1][0];

	//int i = 0;
	//while (raw) {	// ԭ�ϼ�ֵ
	//	if (raw & 1) ans += value_table[i - 1][1] - value_table[i - 1][0];
	//	raw >>= 1;
	//	i++;
	//}
	while (raw) {
		ans += (raw & 1);
		raw >>= 1;
	}
	return ans;
}


Worktop::~Worktop() {}

