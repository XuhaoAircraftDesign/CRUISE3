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
	void EngineCalculate(double t);  // ���㷢������ز���
	void savedata(double t);

public:
	inline double getthrust() const { return thrust; }  
	inline double getmass() const { return mass; }   
	inline double getJx() const { return Jx; }
	inline double getJy() const { return Jy; }
	inline double getJz() const { return Jz; }
	inline int getmode() const { return enginemode; }

private:
	double thrust;              // ����������
	double mass;                // ����
	double Jx;                  // ת������
	double Jy;
	double Jz;
	int enginemode;             // 1- ������������ 0-�������ػ�

	string FileName;
	ofstream outFile;
};

