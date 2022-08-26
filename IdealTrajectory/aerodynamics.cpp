#include "aerodynamics.h"

// ����
aerodynamics::aerodynamics() : FileName("./Autopilot/Coe.txt"), outFile(FileName, std::ios::out)
{
	// ����ϵ��-----------------------------------------------------------
	// ����ͨ��
	a_wz = 0;         
	a_alpha = 0;        
	a_delta_e = 0;    
	b_alpha = 0;      
	b_delta_e = 0;    

	// ƫ��ͨ��
	a_wx = 0;
	a_wy = 0;		
	a_beta = 0;		
	a_delta_r = 0;
	a_delta_a = 0;
	b_beta = 0;      
	b_delta_r = 0;   

	// ��תͨ��
	c_wx = 0;
	c_wy = 0;
	c_delta_a = 0;
	c_delta_r = 0;
	c_beta = 0;

	X = 0;
	Y = 0;
	Z = 0;
	//if (outFile.fail()) std::cerr << "Coe.txt ��ʧ�ܣ�" << std::endl;
}

// ����
aerodynamics::~aerodynamics()
{
	outFile.close();
}

// ������������������
void aerodynamics::AeroForce(double delta_e, double qbar, double alpha)
{	
	// ����ϵ����ϵ��
	double Cx = Cx0 + Cx_alphabeta  * alpha * alpha * 57.3 * 57.3;
	double Cy = Cy0 + Cy_alpha * alpha * 57.3 + Cy_delta_e * 57.3 * delta_e;
	
	// ����ϵ�������� 
	X = -Cx * Sref * qbar;              
	Y = Cy * Sref * qbar;
	Z = 0;
	return;
}

// ����ϵ��
void aerodynamics::DynamicCoefficient(double t, double qbar, double V, double Ma, double height, double P, double mass)
{
	a_wz = -mz_wz * 57.3 * qbar * Sref * Lref * Lref / Jz0 / V;
	a_alpha = -mz_alpha * 57.3 * qbar * Sref * Lref / Jz0;
	a_delta_e = -mz_delta_e * 57.3 * qbar * Sref * Lref / Jz0;
	b_alpha = (P + Cy_alpha * 57.3 * qbar * Sref) / mass / V;
	b_delta_e = Cy_delta_e * 57.3 * qbar * Sref / mass / V;

	a_wx = -my_wx * 57.3 * qbar * Sref * Lref * Lref / Jy0 / V;
	a_wy = -my_wy * 57.3 * qbar * Sref * Lref * Lref / Jy0 / V;
	a_beta = -my_beta * 57.3 * qbar * Sref * Lref / Jy0;
	a_delta_r = -my_delta_r * 57.3 * qbar * Sref * Lref / Jy0;
	a_delta_a = -my_delta_a * 57.3 * qbar * Sref * Lref / Jy0;
	b_beta = (P - Cz_beta * 57.3 * qbar * Sref) / mass / V;
	b_delta_r = -Cz_delta_r * 57.3 * qbar * Sref / mass / V;

	c_wx = -mx_wx * 57.3 * qbar * Sref * Lref * Lref / Jx0 / V / 2.0;
	c_wy = -mx_wy * 57.3 * qbar * Sref * Lref * Lref / Jx0 / V / 2.0;
	c_delta_a = -mx_delta_a * 57.3 * qbar * Sref * Lref / Jx0;
	c_delta_r = -mx_delta_r * 57.3 * qbar * Sref * Lref / Jx0;
	c_beta = -mx_beta * 57.3 * qbar * Sref * Lref / Jx0;

	outFile << t << ",  " << V << ",  " <<  Ma << ",  " <<height << ",  " << qbar << ",  " 
			<< a_wz << ",  " << a_alpha << ",  " << a_delta_e << ",  " << b_alpha << ",  " << b_delta_e << ",  "
			<< a_wx << ",  " << a_wy << ",  " << a_delta_a << ",  " << a_delta_r << ",  " << a_beta << ",  " << b_beta << ",  " << b_delta_r << ",  "
			<< c_wx << ",  " << c_wy << ",  " << c_delta_a << ",  " << c_delta_r << ",  " << c_beta << std::endl;
	return;
}
