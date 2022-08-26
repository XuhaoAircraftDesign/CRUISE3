#pragma once
/*
* 弹簧刀巡飞弹/无人机末制导三自由度理想弹道 
* Three degree of freedom trajectory simulation of cruise missile
* Date:2022.08.26
* Author:Xuhao from school of aerospace of Beijing Institude of Technology, majoring in aircraft design
* 地面发射系视为惯性坐标系，忽略地球自转、椭圆扁率、视地球模型为扁平地球模型
* 地球引力模型
* 国际标准大气模型
* 发动机模型：螺旋桨电推进发动机
* 空气动力学模型：气动外形与无人机一致, 主升力面在纵向平面, 这款导弹没有方向舵, 所以偏航通道内没有控制能力
* 制导系统模型：末制导(电视/图像导引头)
* 控制系统模型：惯导(加速度计、陀螺仪)、气动舵机、过载自动驾驶仪(2回路、3回路)、滚转稳定自动驾驶仪
* 扁平地球巡飞弹三自由度运动方程
*/

#include "guidance.h"     // 制导模型
#include "environment.h"  // 国际标准大气模型 + 地球引力模型
#include "propulsion.h"   // 发动机模型
#include "aerodynamics.h" // 空气动力学模型

/*
制导模型：
1) guidance.h
2) seeker.h 
*/

/*
国际大气标准模块：
1) environment.h
*/

/*
发动机模型：
1) propulsion.h
*/

/*
空气动力学模型：
1) aerodynamics.h
*/

// 弹簧刀巡飞弹/无人机
class missile
{
public:
	missile();
	virtual ~missile();
	void Initialize(int n, const double* x0);
	void setrange(double _range) { ptrguidance->setrange(_range); }     
	void OneStep(int n, double step, double ut_ned=0, double vt_ned=0, double wt_ned=0, 
				 double xt_ned=0, double yt_ned=0, double zt_ned=0);
public:
	// 返回时间
	inline double gettime() const { return time; }                             

	// 返回速度
	inline double getV() const { return V; }							      

	// 返回射程
	inline double getx_ned() const { return x_ned; }                          

	// 返回导弹高度
	inline double gety_ned() const { return y_ned; }                           

	// 返回弹目距离
	inline double getrange() const { return ptrguidance->getrange(); }         

private:
	// 非线性变系数微分方程组数值积分法
	double Gill4(int n, const double* x, double* x_new, double step);

	// 扁平地球巡飞弹三自由度运动方程
	void ThreeDof(int n, const double* x, double* x_dot);    

	// 制导系统
	void Guidance(double ut_ned, double vt_ned, double wt_ned, double xt_ned, double yt_ned, double zt_ned);

	// 保存数据
	void savedata();

private:
	//------------------------------状态变量----------------------------------
	// 发射系位置
	double x_ned;

	// 发射系位置
	double y_ned;

	// 发射系位置
	double z_ned;

	// 发射系速度
	double u_ned;

	// 发射系速度
	double v_ned;

	// 发射系速度
	double w_ned;

	// 速度矢量
	double V;

	// 攻角
	double alpha;

	// 侧滑角
	double beta;

	// 速度滚转角
	double gamma;

	// 升降舵
	double delta_e;

	// 航迹倾角
	double theta;

	// 航迹偏角
	double psiv;

	// 时间
	double time;

	//----------------------------------子系统模型----------------------------------
	// 制导模型
	shared_ptr<guidance> ptrguidance;   // 包含导引头seeker

	// 空气动力模型
	shared_ptr<aerodynamics> ptraerodynamics;

	// 发动机模型
	shared_ptr<propulsion> ptrpropulsion;

	// 标准大气模型
	shared_ptr<environment> ptrenvironment;

	//----------------------------------其他-----------------------------------------
	// 文件名称
	string FileName;

	// 文件路径
	ofstream outFile;
};