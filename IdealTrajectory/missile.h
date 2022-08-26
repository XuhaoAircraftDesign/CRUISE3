#pragma once
/*
* ���ɵ�Ѳ�ɵ�/���˻�ĩ�Ƶ������ɶ����뵯�� 
* Three degree of freedom trajectory simulation of cruise missile
* Date:2022.08.26
* Author:Xuhao from school of aerospace of Beijing Institude of Technology, majoring in aircraft design
* ���淢��ϵ��Ϊ��������ϵ�����Ե�����ת����Բ���ʡ��ӵ���ģ��Ϊ��ƽ����ģ��
* ��������ģ��
* ���ʱ�׼����ģ��
* ������ģ�ͣ����������ƽ�������
* ��������ѧģ�ͣ��������������˻�һ��, ��������������ƽ��, ����û�з����, ����ƫ��ͨ����û�п�������
* �Ƶ�ϵͳģ�ͣ�ĩ�Ƶ�(����/ͼ����ͷ)
* ����ϵͳģ�ͣ��ߵ�(���ٶȼơ�������)����������������Զ���ʻ��(2��·��3��·)����ת�ȶ��Զ���ʻ��
* ��ƽ����Ѳ�ɵ������ɶ��˶�����
*/

#include "guidance.h"     // �Ƶ�ģ��
#include "environment.h"  // ���ʱ�׼����ģ�� + ��������ģ��
#include "propulsion.h"   // ������ģ��
#include "aerodynamics.h" // ��������ѧģ��

/*
�Ƶ�ģ�ͣ�
1) guidance.h
2) seeker.h 
*/

/*
���ʴ�����׼ģ�飺
1) environment.h
*/

/*
������ģ�ͣ�
1) propulsion.h
*/

/*
��������ѧģ�ͣ�
1) aerodynamics.h
*/

// ���ɵ�Ѳ�ɵ�/���˻�
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
	// ����ʱ��
	inline double gettime() const { return time; }                             

	// �����ٶ�
	inline double getV() const { return V; }							      

	// �������
	inline double getx_ned() const { return x_ned; }                          

	// ���ص����߶�
	inline double gety_ned() const { return y_ned; }                           

	// ���ص�Ŀ����
	inline double getrange() const { return ptrguidance->getrange(); }         

private:
	// �����Ա�ϵ��΢�ַ�������ֵ���ַ�
	double Gill4(int n, const double* x, double* x_new, double step);

	// ��ƽ����Ѳ�ɵ������ɶ��˶�����
	void ThreeDof(int n, const double* x, double* x_dot);    

	// �Ƶ�ϵͳ
	void Guidance(double ut_ned, double vt_ned, double wt_ned, double xt_ned, double yt_ned, double zt_ned);

	// ��������
	void savedata();

private:
	//------------------------------״̬����----------------------------------
	// ����ϵλ��
	double x_ned;

	// ����ϵλ��
	double y_ned;

	// ����ϵλ��
	double z_ned;

	// ����ϵ�ٶ�
	double u_ned;

	// ����ϵ�ٶ�
	double v_ned;

	// ����ϵ�ٶ�
	double w_ned;

	// �ٶ�ʸ��
	double V;

	// ����
	double alpha;

	// �໬��
	double beta;

	// �ٶȹ�ת��
	double gamma;

	// ������
	double delta_e;

	// �������
	double theta;

	// ����ƫ��
	double psiv;

	// ʱ��
	double time;

	//----------------------------------��ϵͳģ��----------------------------------
	// �Ƶ�ģ��
	shared_ptr<guidance> ptrguidance;   // ��������ͷseeker

	// ��������ģ��
	shared_ptr<aerodynamics> ptraerodynamics;

	// ������ģ��
	shared_ptr<propulsion> ptrpropulsion;

	// ��׼����ģ��
	shared_ptr<environment> ptrenvironment;

	//----------------------------------����-----------------------------------------
	// �ļ�����
	string FileName;

	// �ļ�·��
	ofstream outFile;
};