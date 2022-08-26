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
	// ����������
	inline double getXbar() const { return X; }
	inline double getYbar() const { return Y; }
	inline double getZbar() const { return Z; }

	// ����������
	void AeroForce(double delta_e, double qbar, double alpha);
	
	// ����ϵ��
	void DynamicCoefficient(double t, double qbar, double V, double Ma, double height, double P, double mass);                                 

private:
	// ��������---------------------------------
	double X;   // ��������
	double Y;   // ��������
	double Z;   // ��������

	// ����ϵ��---------------------------------
	// ����ͨ��
	double a_wz;        
	double a_alpha;      
	double a_delta_e;    
	double b_alpha;     
	double b_delta_e;    

	// ƫ��ͨ��
	double a_wx;
	double a_wy;		
	double a_beta;		
	double a_delta_r;
	double a_delta_a;
	double b_beta;      
	double b_delta_r;  

	// ��תͨ��
	double c_beta;
	double c_wx;       
	double c_wy;
	double c_delta_r;
	double c_delta_a;   

	string FileName;
	ofstream outFile;
};

