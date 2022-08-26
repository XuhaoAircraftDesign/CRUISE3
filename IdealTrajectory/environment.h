#pragma once
#include "constants.h"
#include <cmath>
using namespace environmentPara;

class environment
{
public:
	environment();
	virtual ~environment();

public:
	void ISACalculate(double altitude, double Vt);   // ���ݵ�ǰ�߶Ⱥ��ٶȼ������ģ�Ͳ���

public:
	inline double getMa() const { return Ma; }       // ���������
	inline double getqbar() const { return qbar; }    // ���ض�ѹ
	inline double getrou() const { return rou; }       // ���ؿ����ܶ�
	inline double getgrav() const { return grav; }      // �����������ٶ�

private:
	// ����ģ�Ͳ���
	double temperature;        // �����¶�
	double Ma;                 // �����
	double qbar;               // ��ѹ
	double rou;                // �����ܶ�
	double grav;               // �������ٶ�
};


