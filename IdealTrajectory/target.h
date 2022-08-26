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
	// ����ϵλ��
	double xt_ned;

	// ����ϵλ��
	double yt_ned;

	// ����ϵλ��
	double zt_ned;

	// ����ϵ�ٶ�
	double ut_ned;

	// ����ϵ�ٶ�
	double vt_ned;

	// ����ϵ�ٶ�
	double wt_ned;

	// ʱ��
	double t;

	// ��Ͳ��������
	double Nty;

	// ��Ͳ��������
	double Ntz;

	// �����ļ�����
	string FileName;

	// �����ļ�·��
	ofstream outFile;
};

