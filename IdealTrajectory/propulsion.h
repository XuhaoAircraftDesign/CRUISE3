#pragma once
#include "constants.h"
#include <fstream>
#include <string>
using namespace propulsionPara;
using namespace std;

class propulsion
{
public:
	propulsion();
	virtual ~propulsion();

public:
	void EngineCalculate(double t);  // 计算发动机相关参数
	void savedata(double t);

public:
	inline double getthrust() const { return thrust; }  
	inline double getmass() const { return mass; }   
	inline double getJx() const { return Jx; }
	inline double getJy() const { return Jy; }
	inline double getJz() const { return Jz; }
	inline int getmode() const { return enginemode; }

private:
	double thrust;              // 发动机推力
	double mass;                // 质量
	double Jx;                  // 转动惯量
	double Jy;
	double Jz;
	int enginemode;             // 1- 发动机工作， 0-发动机关机

	string FileName;
	ofstream outFile;
};

