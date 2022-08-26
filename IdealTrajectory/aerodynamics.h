#pragma once
#include "constants.h"
#include <fstream>
#include <string>

using namespace aerodynamicsPara;
using namespace missilePara;
using namespace propulsionPara;
using namespace std;

class aerodynamics
{
public:
	aerodynamics();
	virtual ~aerodynamics();

public:
	// 返回气动力
	inline double getXbar() const { return X; }
	inline double getYbar() const { return Y; }
	inline double getZbar() const { return Z; }

	// 计算气动力
	void AeroForce(double delta_e, double qbar, double alpha);
	
	// 动力系数
	void DynamicCoefficient(double t, double qbar, double V, double Ma, double height, double P, double mass);                                 

private:
	// 空气动力---------------------------------
	double X;   // 空气动力
	double Y;   // 空气动力
	double Z;   // 空气动力

	// 动力系数---------------------------------
	// 俯仰通道
	double a_wz;        
	double a_alpha;      
	double a_delta_e;    
	double b_alpha;     
	double b_delta_e;    

	// 偏航通道
	double a_wx;
	double a_wy;		
	double a_beta;		
	double a_delta_r;
	double a_delta_a;
	double b_beta;      
	double b_delta_r;  

	// 滚转通道
	double c_beta;
	double c_wx;       
	double c_wy;
	double c_delta_r;
	double c_delta_a;   

	string FileName;
	ofstream outFile;
};

