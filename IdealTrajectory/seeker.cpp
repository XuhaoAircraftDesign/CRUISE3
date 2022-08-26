#include "seeker.h"
#include <cmath>

seeker::seeker()
{
    range = 0;
    range_dot = 0;

    qa = 0;
    qb = 0;

    dqa = 0;
    dqb = 0;

    for (int i = 0; i != 3; i++)
    {
        for (int j = 0; j != 3; j++)
        {
            Cn_los2I[i][j] = 0;
        }
    }
}

seeker::~seeker()
{

}

// ��Ŀ����˶���ϵ����
void seeker::RelativeMotion(double u_ned, double v_ned, double w_ned, double x_ned, double y_ned, double z_ned,
    double ut_ned, double vt_ned, double wt_ned, double xt_ned, double yt_ned, double zt_ned)
{
    // ����ϵλ�ò�
    double xr = xt_ned - x_ned;
    double yr = yt_ned - y_ned;
    double zr = zt_ned - z_ned;

    // ����ϵ�ٶȲ�
    double vxr = ut_ned - u_ned;
    double vyr = vt_ned - v_ned;
    double vzr = wt_ned - w_ned;

    // ���λ�ü��仯��
    range = sqrt(xr * xr + yr * yr + zr * zr);
    range_dot = (xr * vxr + yr * vyr + zr * vzr) / range;

    // ���߽�
    qa = atan2(yr, sqrt(xr * xr + zr * zr));
    qb = -atan2(zr, xr);

    // ����ϵ��������ϵ����
    Cn_los2I[0][0] = cos(qa) * cos(qb);
    Cn_los2I[0][1] = -sin(qa) * cos(qb);
    Cn_los2I[0][2] = sin(qb);
    Cn_los2I[1][0] = sin(qa);
    Cn_los2I[1][1] = cos(qa);
    Cn_los2I[1][2] = 0;
    Cn_los2I[2][0] = -cos(qa) * sin(qb);
    Cn_los2I[2][1] = sin(qa) * sin(qb);
    Cn_los2I[2][2] = cos(qb);

    // ���߽��ٶ�,��������ϵ��Թ�������ϵ���ٶ�������ϵͶӰ
    dqa = ((xr * xr + zr * zr) * vyr - yr * (xr * vxr + zr * vzr)) / (range * range * sqrt(xr * xr + zr * zr));
    dqb = (zr * vxr - xr * vzr) / (xr * xr + zr * zr);

    return;
}

void seeker::getCn_los2I(double _Cn_los2I[3][3]) const
{
    for (int i = 0; i != 3; i++)
    {
        for (int j = 0; j != 3; j++)
        {
            _Cn_los2I[i][j] = Cn_los2I[i][j];
        }
    }
    return;
}

