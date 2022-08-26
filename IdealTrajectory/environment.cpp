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

// 根据当前高度和速度计算大气模型参数
void environment::ISACalculate(double altitude, double Vt)
{
	// 当地温度
	if (altitude >= 11000.0)
	{
		temperature = 216.65;
	}
	else
	{
		temperature = T0 - 0.0065 * altitude;
	}

	rou = rou0 * exp((-g0 / (287.05 * temperature)) * altitude);  // 空气密度
	Ma = Vt / sqrt(1.4 * 287.05 * temperature);                   // 马赫数
	qbar = 0.5 * rou * Vt * Vt;                                    // 动压
	grav = g0 * (Re * Re / ((Re + altitude) * (Re + altitude)));  // 重力加速度
	return;
}

