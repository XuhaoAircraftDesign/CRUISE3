#pragma once

// 电视导引头
class seeker
{
public:
	seeker();
	virtual ~seeker();

public:
	void RelativeMotion(double u_ned, double v_ned, double w_ned, double x_ned, double y_ned, double z_ned,
		double ut_ned, double vt_ned, double wt_ned, double xt_ned, double yt_ned, double zt_ned);
	inline void setrange(double _range) { range = _range; }			              // 初始弹目距离
	double getrange() const { return range; }
	double getdqa() const { return dqa; }
	double getdqb() const { return dqb; }
	void getCn_los2I(double _Cn_los2I[3][3]) const;

public:
	double range;		 // 弹目距离
	double range_dot;    // 弹目距离变化率

	double qa;           // 视线角
	double qb;           // 视线角

	double Cn_los2I[3][3];     // 视线系到惯性系旋转矩阵

	double dqa;          // 视线角速度, 相对于惯性系, 视线坐标系下
	double dqb;			 // 视线角速度, 相对于惯性系, 视线坐标系下
};

