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
	void ISACalculate(double altitude, double Vt);   // 根据当前高度和速度计算大气模型参数

public:
	inline double getMa() const { return Ma; }       // 返回马赫数
	inline double getqbar() const { return qbar; }    // 返回动压
	inline double getrou() const { return rou; }       // 返回空气密度
	inline double getgrav() const { return grav; }      // 返回重力加速度

private:
	// 大气模型参数
	double temperature;        // 当地温度
	double Ma;                 // 马赫数
	double qbar;               // 动压
	double rou;                // 空气密度
	double grav;               // 重力加速度
};


