#include "guidance.h"
#include <iostream>

// ����
guidance::guidance() : ptrseeker(new seeker), FileName("./Trajectory/guidance.txt"), outFile(FileName, std::ios::out)
{
    N1 = 4.0;
    N2 = 4.0;

    ayc = 0;
    azc = 0;
    gammac = 0;

    if (outFile.fail()) std::cerr << "guidance.txt ��ʧ�ܣ�" << std::endl;
}

// ����
guidance::~guidance()
{
    outFile.close();
}

// ��������
void guidance::savedata(double t)
{
    outFile << t << ",  " << ayc << ",  " << azc<< ",  " <<  ptrseeker->getrange() << std::endl;
}


// ���������Ƶ���
void guidance::PN(double V, double g, double theta)
{  
    // �ӵ���ͷ��ȡ���߽�����
    double dqa = ptrseeker->getdqa();
    double dqb = ptrseeker->getdqb();

    // ����ָ��, ����ϵ��
    ayc = N1 * V * dqa + g*cos(theta);
    azc = N2 * V * dqb;

    // ����ָ���޷�
    if (fabs(ayc) >= 25 * g) ayc = ayc / fabs(ayc) * 25 * g;
    if (fabs(azc) >= 25 * g) azc = azc / fabs(azc) * 25 * g;

    // �ٶȹ�ת��ָ���޷�
    gammac = -atan2(azc, ayc);
    if (fabs(gammac) >= 45 / 57.3) gammac = gammac / fabs(gammac) * 45 / 57.3;

    return;
}

// ִ�е���ͷ����
void guidance::seekerCall(double u_ned, double v_ned, double w_ned, double x_ned, double y_ned, double z_ned,
    double ut_ned, double vt_ned, double wt_ned, double xt_ned, double yt_ned, double zt_ned)
{
    ptrseeker->RelativeMotion(u_ned, v_ned, w_ned, x_ned, y_ned, z_ned, ut_ned, vt_ned, wt_ned, xt_ned, yt_ned,zt_ned);
    return;
}
