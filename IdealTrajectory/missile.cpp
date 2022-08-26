#include "missile.h"
#include <iostream>

// ���캯��
missile::missile() : ptrguidance(new guidance), 
ptraerodynamics(new aerodynamics), ptrpropulsion(new propulsion), ptrenvironment(new environment),
FileName("./Trajectory/missile.txt"), outFile(FileName, std::ios::out)
{
	//------------------------------״̬����----------------------------------
	// ����ϵλ��
	x_ned = 0;

	// ����ϵλ��
	y_ned = 0;

	// ����ϵλ��
	z_ned = 0;

	// ����ϵλ��
	u_ned = 0;

	// ����ϵλ��
	v_ned = 0;

	// ����ϵλ��
	w_ned = 0;

	// �ٶ�ʸ��
	V = 0;

	// ����
	alpha = 0;

	// �໬��
	beta = 0;

	// �ٶȹ�ת��
	gamma = 0;

	// ������
	delta_e = 0;

	// �������
	theta = 0;

	// ����ƫ��
	psiv = 0;

	// ʱ��
	time = 0;

	// �ж��ļ��Ƿ��
	//if (outFile.fail()) std::cerr << "missile.txt ��ʧ�ܣ�" << std::endl;
}

// ��������
missile::~missile()
{
	// �ر��ļ�
	outFile.close();
}

// ��ʼ������
void missile::Initialize(int n, const double* x0)
{
	// ��ʼ��״̬����
	V = x0[0];
	theta = x0[1];
	psiv = x0[2];
	x_ned = x0[3];
	y_ned = x0[4];
	z_ned = x0[5];
	time = x0[6];
	return;
}

// �ⲿ���ýӿ�
void missile::OneStep(int n, double step, double ut_ned, double vt_ned, double wt_ned, double xt_ned, double yt_ned, double zt_ned)
{
	// ״̬����
	double x[7];        
	double x_new[7];    

	// ��ȡ��ǰ״̬
	x[0] = V;
	x[1] = theta;
	x[2] = psiv;
	x[3] = x_ned;
	x[4] = y_ned;
	x[5] = z_ned;
	x[6] = time;

	// ��ֵ���ָ���״̬����
	double error = Gill4(n, x, x_new, step);   

	// ����״̬���� 
	V = x_new[0];
	theta = x_new[1];
	psiv = x_new[2];
	x_ned = x_new[3];
	y_ned = x_new[4];
	z_ned = x_new[5];
	time = x_new[6];

	// �Ƶ�ϵͳģ��
	Guidance(ut_ned, vt_ned, wt_ned, xt_ned, yt_ned, zt_ned);

	// ���㶯��ϵ��
	ptraerodynamics->DynamicCoefficient(time, ptrenvironment->getqbar(), V, ptrenvironment->getMa(), 
										y_ned, ptrpropulsion->getthrust(), ptrpropulsion->getmass());

	// �˶����ݱ����ļ�
	savedata();     

	return;
}

// �����Ա�ϵ��΢�ַ�������ֵ���ַ�
double missile::Gill4(int n, const double* x, double* x_new, double step)
{
	static const double a[4] = { 1.0 / 6.0, (2.0 - sqrt(2.0)) / 6.0, (2.0 + sqrt(2.0)) / 6.0, 1.0 / 6.0 };
	static const double b[6] = { 0.5, sqrt(0.5) - 0.5, 1.0 - sqrt(0.5), 0, -sqrt(0.5), sqrt(0.5) + 1.0 };

	double x_temp[7];
	double x_dot[4][7];

	this->ThreeDof(n, x, x_dot[0]);
	for (int i = 0; i != n; i++) x_temp[i] = x[i] + step * b[0] * x_dot[0][i];

	this->ThreeDof(n, x_temp, x_dot[1]);
	for (int i = 0; i != n; i++) x_temp[i] = x[i] + step * (b[1] * x_dot[0][i] + b[2] * x_dot[1][i]);

	this->ThreeDof(n, x_temp, x_dot[2]);
	for (int i = 0; i != n; i++) x_temp[i] = x[i] + step * (b[3] * x_dot[0][i] + b[4] * x_dot[1][i] + b[5] * x_dot[2][i]);

	this->ThreeDof(n, x_temp, x_dot[3]);
	for (int i = 0; i != n; i++) x_new[i] = x[i] + step * (a[0] * x_dot[0][i] + a[1] * x_dot[1][i] + a[2] * x_dot[2][i] + a[3] * x_dot[3][i]);
	return 0;
}

// ���ɵ�Ѳ�ɵ�/���˻������ɶ��˶�����
void missile::ThreeDof(int n, const double* x, double* x_dot)
{
	// ��ȡ��ǰʱ��״̬����------------------------------------------------------------------------------
	V = x[0];
	theta = x[1];
	psiv = x[2];
	x_ned = x[3];
	y_ned = x[4];
	z_ned = x[5];
	time = x[6];

	// ������ز���-------------------------------------------------------------------------------------
	u_ned = V * cos(theta) * cos(psiv);
	v_ned = V * sin(theta);
	w_ned = -V * cos(theta) * sin(psiv);

	// �Ƶ�ϵͳ----------------------------------------------------------------------------------------
	double ayc = ptrguidance->getayc();
	double gammac = ptrguidance->getgammac();

	// ���ʱ�׼����ģ��--------------------------------------------------------------------------------
	ptrenvironment->ISACalculate(y_ned, V);
	double g = ptrenvironment->getgrav();
	double Ma = ptrenvironment->getMa();
	double qbar = ptrenvironment->getqbar();

	// ������ģ��--------------------------------------------------------------------------------------
	ptrpropulsion->EngineCalculate(time);
	double thrust = ptrpropulsion->getthrust();
	double mass = ptrpropulsion->getmass();

	// ˲ʱƽ�����----------------------------------------------------------------------------------
	double Y_0 = qbar * Sref * Cy0;
	double Y_alpha = qbar * Sref * Cy_alpha * 57.3;   // �������Լ����PDF˵��
	double Y_delta_e = qbar * Sref * Cy_delta_e * 57.3;
	alpha = (ayc * mass + Y_delta_e * mz0 / mz_delta_e / 57.3 - Y_0) / (thrust + Y_alpha - Y_delta_e * mz_alpha / mz_delta_e);
	beta = 0;
	gamma = gammac;
	delta_e = -(mz0 + mz_alpha * 57.3 * alpha) / mz_delta_e / 57.3;

	// ��������ѧģ��---------------------------------------------------------------------------------
	ptraerodynamics->AeroForce(delta_e, qbar, alpha);   									
	double Xbar = ptraerodynamics->getXbar();
	double Ybar = ptraerodynamics->getYbar();
	double Zbar = ptraerodynamics->getZbar();

	// �����Ա�ϵ��΢�ַ�����--------------------------------------------------------------------------
	// ţ�ٶ���ѧ->���Ķ���ѧ(����ϵ�����ٶȵ���)  
	double V_dot = (Xbar + thrust * cos(alpha)) / mass - g * sin(theta);
	double theta_dot = (Ybar * cos(gamma) + thrust * sin(alpha) * cos(gamma)) / mass / V - g / V * cos(theta);
	double psiv_dot = -(Ybar * sin(gamma) + thrust * sin(alpha) * sin(gamma)) / mass / V / cos(theta);

	// ��������->�����˶�ѧ(����ϵ��λ�õ���) 
	double x_ned_dot = u_ned;
	double y_ned_dot = v_ned;
	double z_ned_dot = w_ned;

	// ���µ�ǰʱ��״̬��������--------------------------------------------------------------------------
	x_dot[0] = V_dot;
	x_dot[1] = theta_dot;
	x_dot[2] = psiv_dot;
	x_dot[3] = x_ned_dot;
	x_dot[4] = y_ned_dot;
	x_dot[5] = z_ned_dot;
	x_dot[6] = 1.0;
	return;
}

// �Ƶ�ϵͳ
void missile::Guidance(double ut_ned, double vt_ned, double wt_ned, double xt_ned, double yt_ned, double zt_ned)
{
	// ���ӵ���ͷ seeker
	ptrguidance->seekerCall(u_ned, v_ned, w_ned, x_ned, y_ned, z_ned, ut_ned, vt_ned, wt_ned, xt_ned, yt_ned, zt_ned);

	// ������������
	ptrguidance->PN(V, ptrenvironment->getgrav(), theta);

	return;
}

// ��������
void missile::savedata()
{
	outFile << time << ", " << V << ", " << alpha << ", " << beta << ", "
		<< theta << ", " << psiv << ", "
		<< x_ned << ", " << y_ned << ", " << z_ned << ", "<< delta_e << ", "<<gamma
		<< endl;
	ptrguidance->savedata(time);
	ptrpropulsion->savedata(time);
	return;
}