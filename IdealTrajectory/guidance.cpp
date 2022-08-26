#include "guidance.h"
#include <iostream>

// 构造
guidance::guidance() : ptrseeker(new seeker), FileName("./Trajectory/guidance.txt"), outFile(FileName, std::ios::out)
{
    N1 = 4.0;
    N2 = 4.0;

    ayc = 0;
    azc = 0;
    gammac = 0;

    if (outFile.fail()) std::cerr << "guidance.txt 打开失败！" << std::endl;
}

// 析构
guidance::~guidance()
{
    outFile.close();
}

// 保存数据
void guidance::savedata(double t)
{
    outFile << t << ",  " << ayc << ",  " << azc<< ",  " <<  ptrseeker->getrange() << std::endl;
}


// 比例导引制导律
void guidance::PN(double V, double g, double theta)
{  
    // 从导引头获取视线角速率
    double dqa = ptrseeker->getdqa();
    double dqb = ptrseeker->getdqb();

    // 过载指令, 视线系下
    ayc = N1 * V * dqa + g*cos(theta);
    azc = N2 * V * dqb;

    // 过载指令限幅
    if (fabs(ayc) >= 25 * g) ayc = ayc / fabs(ayc) * 25 * g;
    if (fabs(azc) >= 25 * g) azc = azc / fabs(azc) * 25 * g;

    // 速度滚转角指令限幅
    gammac = -atan2(azc, ayc);
    if (fabs(gammac) >= 45 / 57.3) gammac = gammac / fabs(gammac) * 45 / 57.3;

    return;
}

// 执行导引头功能
void guidance::seekerCall(double u_ned, double v_ned, double w_ned, double x_ned, double y_ned, double z_ned,
    double ut_ned, double vt_ned, double wt_ned, double xt_ned, double yt_ned, double zt_ned)
{
    ptrseeker->RelativeMotion(u_ned, v_ned, w_ned, x_ned, y_ned, z_ned, ut_ned, vt_ned, wt_ned, xt_ned, yt_ned,zt_ned);
    return;
}
