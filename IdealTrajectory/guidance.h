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
	inline void setrange(double _range) { ptrseeker->setrange(_range); }		// ��ʼ��Ŀ����
	inline double getrange() const { return ptrseeker->range; }				      // ���ص�Ŀ����
	inline double getayc() const { return ayc; }					              // �����������ָ��
	inline double getazc() const { return azc; }                                  // �����������ָ��
	inline double getgammac() const { return gammac; }                            // �ٶȹ�ת��ָ�� 

public:

	// ���������Ƶ���
	void PN(double V, double g, double theta);

	// ִ�е���ͷ����
	void seekerCall(double u_ned, double v_ned, double w_ned, double x_ned, double y_ned, double z_ned,
		double ut_ned, double vt_ned, double wt_ned, double xt_ned, double yt_ned, double zt_ned);

	// ��������
	void savedata(double t);

private: 
	double N1;           // ������
	double N2;			 // ������

	double ayc;          // �������ָ��
	double azc;          // �������ָ��
	double gammac;

	shared_ptr<seeker> ptrseeker;

	string FileName;	 // �ļ���
	ofstream outFile;    // �ļ�·��
};

