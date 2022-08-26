#include "missile.h"
#include <iostream>

// 构造函数
missile::missile() : ptrguidance(new guidance), 
ptraerodynamics(new aerodynamics), ptrpropulsion(new propulsion), ptrenvironment(new environment),
FileName("./Trajectory/missile.txt"), outFile(FileName, std::ios::out)
{
	//------------------------------状态变量----------------------------------
	// 发射系位置
	x_ned = 0;

	// 发射系位置
	y_ned = 0;

	// 发射系位置
	z_ned = 0;

	// 发射系位置
	u_ned = 0;

	// 发射系位置
	v_ned = 0;

	// 发射系位置
	w_ned = 0;

	// 速度矢量
	V = 0;

	// 攻角
	alpha = 0;

	// 侧滑角
	beta = 0;

	// 速度滚转角
	gamma = 0;

	// 升降舵
	delta_e = 0;

	// 航迹倾角
	theta = 0;

	// 航迹偏角
	psiv = 0;

	// 时间
	time = 0;

	// 判断文件是否打开
	//if (outFile.fail()) std::cerr << "missile.txt 打开失败！" << std::endl;
}

// 析构函数
missile::~missile()
{
	// 关闭文件
	outFile.close();
}

// 初始化函数
void missile::Initialize(int n, const double* x0)
{
	// 初始化状态向量
	V = x0[0];
	theta = x0[1];
	psiv = x0[2];
	x_ned = x0[3];
	y_ned = x0[4];
	z_ned = x0[5];
	time = x0[6];
	return;
}

// 外部调用接口
void missile::OneStep(int n, double step, double ut_ned, double vt_ned, double wt_ned, double xt_ned, double yt_ned, double zt_ned)
{
	// 状态向量
	double x[7];        
	double x_new[7];    

	// 获取当前状态
	x[0] = V;
	x[1] = theta;
	x[2] = psiv;
	x[3] = x_ned;
	x[4] = y_ned;
	x[5] = z_ned;
	x[6] = time;

	// 数值积分更新状态向量
	double error = Gill4(n, x, x_new, step);   

	// 更新状态向量 
	V = x_new[0];
	theta = x_new[1];
	psiv = x_new[2];
	x_ned = x_new[3];
	y_ned = x_new[4];
	z_ned = x_new[5];
	time = x_new[6];

	// 制导系统模型
	Guidance(ut_ned, vt_ned, wt_ned, xt_ned, yt_ned, zt_ned);

	// 计算动力系数
	ptraerodynamics->DynamicCoefficient(time, ptrenvironment->getqbar(), V, ptrenvironment->getMa(), 
										y_ned, ptrpropulsion->getthrust(), ptrpropulsion->getmass());

	// 运动数据保存文件
	savedata();     

	return;
}

// 非线性变系数微分方程组数值积分法
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

// 弹簧刀巡飞弹/无人机三自由度运动方程
void missile::ThreeDof(int n, const double* x, double* x_dot)
{
	// 获取当前时刻状态向量------------------------------------------------------------------------------
	V = x[0];
	theta = x[1];
	psiv = x[2];
	x_ned = x[3];
	y_ned = x[4];
	z_ned = x[5];
	time = x[6];

	// 计算相关参数-------------------------------------------------------------------------------------
	u_ned = V * cos(theta) * cos(psiv);
	v_ned = V * sin(theta);
	w_ned = -V * cos(theta) * sin(psiv);

	// 制导系统----------------------------------------------------------------------------------------
	double ayc = ptrguidance->getayc();
	double gammac = ptrguidance->getgammac();

	// 国际标准大气模型--------------------------------------------------------------------------------
	ptrenvironment->ISACalculate(y_ned, V);
	double g = ptrenvironment->getgrav();
	double Ma = ptrenvironment->getMa();
	double qbar = ptrenvironment->getqbar();

	// 发动机模型--------------------------------------------------------------------------------------
	ptrpropulsion->EngineCalculate(time);
	double thrust = ptrpropulsion->getthrust();
	double mass = ptrpropulsion->getmass();

	// 瞬时平衡假设----------------------------------------------------------------------------------
	double Y_0 = qbar * Sref * Cy0;
	double Y_alpha = qbar * Sref * Cy_alpha * 57.3;   // 气动特性计算见PDF说明
	double Y_delta_e = qbar * Sref * Cy_delta_e * 57.3;
	alpha = (ayc * mass + Y_delta_e * mz0 / mz_delta_e / 57.3 - Y_0) / (thrust + Y_alpha - Y_delta_e * mz_alpha / mz_delta_e);
	beta = 0;
	gamma = gammac;
	delta_e = -(mz0 + mz_alpha * 57.3 * alpha) / mz_delta_e / 57.3;

	// 空气动力学模型---------------------------------------------------------------------------------
	ptraerodynamics->AeroForce(delta_e, qbar, alpha);   									
	double Xbar = ptraerodynamics->getXbar();
	double Ybar = ptraerodynamics->getYbar();
	double Zbar = ptraerodynamics->getZbar();

	// 非线性变系数微分方程组--------------------------------------------------------------------------
	// 牛顿动力学->质心动力学(弹体系下线速度导数)  
	double V_dot = (Xbar + thrust * cos(alpha)) / mass - g * sin(theta);
	double theta_dot = (Ybar * cos(gamma) + thrust * sin(alpha) * cos(gamma)) / mass / V - g / V * cos(theta);
	double psiv_dot = -(Ybar * sin(gamma) + thrust * sin(alpha) * sin(gamma)) / mass / V / cos(theta);

	// 导航方程->质心运动学(发射系下位置导数) 
	double x_ned_dot = u_ned;
	double y_ned_dot = v_ned;
	double z_ned_dot = w_ned;

	// 更新当前时刻状态向量导数--------------------------------------------------------------------------
	x_dot[0] = V_dot;
	x_dot[1] = theta_dot;
	x_dot[2] = psiv_dot;
	x_dot[3] = x_ned_dot;
	x_dot[4] = y_ned_dot;
	x_dot[5] = z_ned_dot;
	x_dot[6] = 1.0;
	return;
}

// 制导系统
void missile::Guidance(double ut_ned, double vt_ned, double wt_ned, double xt_ned, double yt_ned, double zt_ned)
{
	// 电视导引头 seeker
	ptrguidance->seekerCall(u_ned, v_ned, w_ned, x_ned, y_ned, z_ned, ut_ned, vt_ned, wt_ned, xt_ned, yt_ned, zt_ned);

	// 补偿比例导引
	ptrguidance->PN(V, ptrenvironment->getgrav(), theta);

	return;
}

// 保存数据
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