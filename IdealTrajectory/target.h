#pragma once
#include <memory>
#include <fstream>
#include <string>
using namespace std;

class target
{
public:
	target();
	virtual ~target();
	void Initialize(int n, const double* x0, double _Nty=0, double _Ntz=0);
	void OneStep(int n, double h);                  

public:
	inline double getut_ned() const { return ut_ned; }
	inline double getvt_ned() const { return vt_ned; }
	inline double getwt_ned() const { return wt_ned; }
	inline double getxt_ned() const { return xt_ned; }
	inline double getyt_ned() const { return yt_ned; }
	inline double getzt_ned() const { return zt_ned; }
	 
private:
	void ThreeDof(int n, const double* x, double* x_dot, double h); 									
	double Gill4(int n, const double* x, double* x_new, double h);
	void SaveIntostdvector();

private:
	// 地面系位置
	double xt_ned;

	// 地面系位置
	double yt_ned;

	// 地面系位置
	double zt_ned;

	// 地面系速度
	double ut_ned;

	// 地面系速度
	double vt_ned;

	// 地面系速度
	double wt_ned;

	// 时间
	double t;

	// 滚筒机动过载
	double Nty;

	// 滚筒机动过载
	double Ntz;

	// 保存文件名称
	string FileName;

	// 保存文件路径
	ofstream outFile;
};

