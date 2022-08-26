#pragma once
#include <fstream>
#include <string>
#include "seeker.h"
#include <memory>
using namespace std;

class guidance
{
public:
	guidance();
	virtual ~guidance();

public:
	inline void setrange(double _range) { ptrseeker->setrange(_range); }		// 初始弹目距离
	inline double getrange() const { return ptrseeker->range; }				      // 返回弹目距离
	inline double getayc() const { return ayc; }					              // 返回纵向过载指令
	inline double getazc() const { return azc; }                                  // 返回纵向过载指令
	inline double getgammac() const { return gammac; }                            // 速度滚转角指令 

public:

	// 比例导引制导律
	void PN(double V, double g, double theta);

	// 执行导引头功能
	void seekerCall(double u_ned, double v_ned, double w_ned, double x_ned, double y_ned, double z_ned,
		double ut_ned, double vt_ned, double wt_ned, double xt_ned, double yt_ned, double zt_ned);

	// 保存数据
	void savedata(double t);

private: 
	double N1;           // 导航比
	double N2;			 // 导航比

	double ayc;          // 纵向过载指令
	double azc;          // 侧向过载指令
	double gammac;

	shared_ptr<seeker> ptrseeker;

	string FileName;	 // 文件名
	ofstream outFile;    // 文件路径
};

