#include "environment.h"

environment::environment()
{
	Ma = 0;
	qbar = 0;
	rou = 0;
	temperature = 0;
	grav = 0;
}

environment::~environment()
{

}

// ���ݵ�ǰ�߶Ⱥ��ٶȼ������ģ�Ͳ���
void environment::ISACalculate(double altitude, double Vt)
{
	// �����¶�
	if (altitude >= 11000.0)
	{
		temperature = 216.65;
	}
	else
	{
		temperature = T0 - 0.0065 * altitude;
	}

	rou = rou0 * exp((-g0 / (287.05 * temperature)) * altitude);  // �����ܶ�
	Ma = Vt / sqrt(1.4 * 287.05 * temperature);                   // �����
	qbar = 0.5 * rou * Vt * Vt;                                    // ��ѹ
	grav = g0 * (Re * Re / ((Re + altitude) * (Re + altitude)));  // �������ٶ�
	return;
}

