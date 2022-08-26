#pragma once

namespace missilePara
{
	// 巡飞弹结构参数
	static const double Sref = 0.0064;   // 特征面积
	static const double Lref = 0.74;     // 特征长度
}

namespace environmentPara
{
	// 国际标准大气模型+地球引力模型
	static const double g0 = 9.80665;   // 海平面重力加速度
	static const double rou0 = 1.225;   // 海平面空气密度
	static const double Re = 6371000;   // 海平面地球平均半径
	static const double T0 = 288.15;    // 海平面温度
}

namespace propulsionPara
{
	// 螺旋桨电推进发动机
	static const double mass0 = 4;     // mass
	static const double thrust0 = 15;  // thrust
	static const double Jx0 = 0.04;     // Jx
	static const double Jy0 = 0.4;      // Jy
	static const double Jz0 = 0.35;     // Jz
}

namespace aerodynamicsPara
{
	// 巡飞弹气动外形与无人机一致, 主升力面在纵向平面, 这款导弹没有方向舵，所以偏航通道内没有控制能力, 控制方式为 BTT-90
	// Cx
	static const double Cx0 = 0.7822;
	static const double Cx_alphabeta = 0.02131;

	// Cy 
	static const double Cy0 = 6.316;
	static const double Cy_alpha = 2.247;
	static const double Cy_delta_e = 0.5432;

	// Cz
	static const double Cz0 = 0.0289;
	static const double Cz_beta = -0.09026;
	static const double Cz_delta_r = 0;

	// mx 
	static const double mx0 = 0.299;
	static const double mx_beta = 0.01096;
	static const double mx_delta_a = 0.1661;
	static const double mx_delta_r = 0;
	static const double mx_wx = -8.8;
	static const double mx_wy = -0.03;

	// my
	static const double my0 = 0.03592;
	static const double my_beta = -0.1038;
	static const double my_delta_a = 0;
	static const double my_delta_r = 0;
	static const double my_wx = -0.9;
	static const double my_wy = -1;

	// mz
	static const double mz0 = -1.003;
	static const double mz_alpha = -0.2839;
	static const double mz_delta_e = 0.1519;
	static const double mz_wz = -16.5;
}







