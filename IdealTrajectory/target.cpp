#include "target.h"

// 构造函数
target::target() : FileName("./Trajectory/Target.txt"), outFile(FileName, std::ios::out)
{
	xt_ned = 0;
	yt_ned = 0;
	zt_ned = 0;
	ut_ned = 0;
	vt_ned = 0;
	wt_ned = 0;
	t = 0;
	Nty = 0;
	Ntz = 0;

	//if (outFile.fail()) std::cerr << "Target.txt 打开失败！" << std::endl;
}

void target::Initialize(int n, const double* x0, double _Nty, double _Ntz)
{
	double Vt = x0[0];
	double thetat = x0[1];
	double psivt = x0[2];
	xt_ned = x0[3];
	yt_ned = x0[4];
	zt_ned = x0[5];
	ut_ned = Vt * cos(thetat) * cos(psivt);
	vt_ned = Vt * sin(thetat);
	wt_ned = -Vt * cos(thetat) * sin(psivt);
	t = x0[6];
	Nty = _Nty;
	Ntz = _Ntz;

	return;
}

// 析构函数
target::~target()
{
	outFile.close();
}

// 运行逻辑函数
void target::OneStep(int n, double h)
{
	// 定义状态向量
	double x[7];
	double x_new[7];

	// 获取状态成员,为数值积分准备
	x[0] = ut_ned;
	x[1] = vt_ned;
	x[2] = wt_ned;
	x[3] = xt_ned;
	x[4] = yt_ned;
	x[5] = zt_ned;
	x[6] = t;

	// 数值积分得到下一步状态向量
	double error = Gill4(n, x, x_new, h);   // 变步长计算需判断误差 error 是否满足要求 ... ...

	// 更新状态向量
	ut_ned = x_new[0];
	vt_ned = x_new[1];
	wt_ned = x_new[2];
	xt_ned = x_new[3];
	yt_ned = x_new[4];
	zt_ned = x_new[5];
	t = x_new[6];

	// 每一步仿真数据保存到动态数组中
	SaveIntostdvector();
	return;
}

// 三自由度运动方程
void target::ThreeDof(int n, const double* x, double* x_dot, double h)
{
	// 获取当前时刻状态向量
	ut_ned = x[0];
	vt_ned = x[1];
	wt_ned = x[2];
	xt_ned = x[3];
	yt_ned = x[4];
	zt_ned = x[5];
	t = x[6];

	// 目标机动模型
	double ax = 0;
	double ay = 0;  
	double az = 0;  

	// 地理系下质心动力学
	double ut_ned_dot = ax;
	double vt_ned_dot = ay;
	double wt_ned_dot = az;

	// 地理系下质心运动学
	double xt_ned_dot = ut_ned;
	double yt_ned_dot = vt_ned;
	double zt_ned_dot = wt_ned;

	// 更新当前时刻状态向量导数
	x_dot[0] = ut_ned_dot;
	x_dot[1] = vt_ned_dot;
	x_dot[2] = wt_ned_dot;
	x_dot[3] = xt_ned_dot;
	x_dot[4] = yt_ned_dot;
	x_dot[5] = zt_ned_dot;
	x_dot[6] = 1.0;
	return;
}

// Gill4 数值积分法
double target::Gill4(int n, const double* x, double* x_new, double h)
{
	static const double a[4] = { 1.0 / 6.0, (2.0 - sqrt(2.0)) / 6.0, (2.0 + sqrt(2.0)) / 6.0, 1.0 / 6.0 };
	static const double b[6] = { 1.0 / 2.0, sqrt(1.0 / 2.0) - 1.0 / 2.0, 1.0 - sqrt(1.0 / 2.0), 0, -sqrt(1.0 / 2.0), sqrt(1.0 / 2.0) + 1.0 };

	double x_temp[7];
	double x_dot[4][7];

	ThreeDof(n, x, x_dot[0], h);
	for (int i = 0; i != n; i++) x_temp[i] = x[i] + h * b[0] * x_dot[0][i];

	ThreeDof(n, x_temp, x_dot[1], h);
	for (int i = 0; i != n; i++) x_temp[i] = x[i] + h * (b[1] * x_dot[0][i] + b[2] * x_dot[1][i]);

	ThreeDof(n, x_temp, x_dot[2], h);
	for (int i = 0; i != n; i++) x_temp[i] = x[i] + h * (b[3] * x_dot[0][i] + b[4] * x_dot[1][i] + b[5] * x_dot[2][i]);

	ThreeDof(n, x_temp, x_dot[3], h);
	for (int i = 0; i != n; i++) x_new[i] = x[i] + h * (a[0] * x_dot[0][i] + a[1] * x_dot[1][i] + a[2] * x_dot[2][i] + a[3] * x_dot[3][i]);
	return 0;
}

// 保存数据std::vector
void target::SaveIntostdvector()
{
	// 目标运动数据
	outFile << t << ",  " << ut_ned << ",  " << vt_ned << ",  " << wt_ned << ",  " << xt_ned << ",  " << yt_ned << ",  " << zt_ned << std::endl;
	return;
}