#pragma once

// ���ӵ���ͷ
class seeker
{
public:
	seeker();
	virtual ~seeker();

public:
	void RelativeMotion(double u_ned, double v_ned, double w_ned, double x_ned, double y_ned, double z_ned,
		double ut_ned, double vt_ned, double wt_ned, double xt_ned, double yt_ned, double zt_ned);
	inline void setrange(double _range) { range = _range; }			              // ��ʼ��Ŀ����
	double getrange() const { return range; }
	double getdqa() const { return dqa; }
	double getdqb() const { return dqb; }
	void getCn_los2I(double _Cn_los2I[3][3]) const;

public:
	double range;		 // ��Ŀ����
	double range_dot;    // ��Ŀ����仯��

	double qa;           // ���߽�
	double qb;           // ���߽�

	double Cn_los2I[3][3];     // ����ϵ������ϵ��ת����

	double dqa;          // ���߽��ٶ�, ����ڹ���ϵ, ��������ϵ��
	double dqb;			 // ���߽��ٶ�, ����ڹ���ϵ, ��������ϵ��
};

