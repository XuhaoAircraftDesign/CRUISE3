#include "missile.h"   // Ѳ�ɵ�
#include "Target.h"    // ����Ŀ��   
#include <iostream>
using namespace std;

// ������ 
int main()
{
    std::cout << "Step 1 : ��ʼ��Ŀ���˶�������" << endl;
    double Vt0 = 10;        
    double thetat0 = 0;    
    double psivt0 = 0;         
    double xt0 = 1000;          
    double yt0 = 0;              
    double zt0 = 300;          
    double tt0 = 0;
    /*
    std::cout << "������Ŀ�굯�����(��λ����/deg)�� " << endl;
    std::cin >> thetat0;
    std::cout << "������Ŀ�굯��ƫ��(��λ����/deg)�� " << endl;
    std::cin >> psivt0;
    std::cout << "������Ŀ��ˮƽλ��(��λ��m)�� " << endl;
    cin >> xt0;
    */
    double x0_target[7];
    x0_target[0] = Vt0;
    x0_target[1] = thetat0 / 57.3;
    x0_target[2] = psivt0 / 57.3;
    x0_target[3] = xt0;
    x0_target[4] = yt0;
    x0_target[5] = zt0;
    x0_target[6] = tt0;
    target Target;
    Target.Initialize(7, x0_target);

    std::cout << "Step 2 : ��ʼ�������˶�������" << endl;
    double V0 = 30;  
    double theta0 = 0;
    double psiv0 = 0;
    double x_ned0 = 0;                    
    double y_ned0 = 100;
    double z_ned0 = 0;  
    double t0 = 0;    
    double x0_missile[7];
    x0_missile[0] = V0;
    x0_missile[1] = theta0;
    x0_missile[2] = theta0;
    x0_missile[3] = x_ned0;
    x0_missile[4] = y_ned0;
    x0_missile[5] = z_ned0;
    x0_missile[6] = t0;
    missile Missile;
    Missile.Initialize(7, x0_missile);

    std::cout << "Step 3 : ��ʼ��������Ŀ��֮����룡" << endl;
    double xr0 = x0_target[3] - x0_missile[3];
    double yr0 = x0_target[4] - x0_missile[4];
    double zr0 = x0_target[5] - x0_missile[5];
    double R = sqrt(xr0 * xr0 + yr0 * yr0 + zr0 * zr0);     // ��Ŀ����
    Missile.setrange(R);              

    std::cout << "Step 4 : ��ʼ����ϣ���ʼ���㣡" << endl;
    while (Missile.gety_ned() >= 0 && R >= 1)
    {
        // Ŀ�����һ����
        Target.OneStep(7, 0.01);

        // ��������һ����
        Missile.OneStep(7, 0.01, Target.getut_ned(), Target.getvt_ned(), Target.getwt_ned(), 
                        Target.getxt_ned(), Target.getyt_ned(), Target.getzt_ned());

        // ��һ������Ŀ����
        R = Missile.getrange();

        // �ն���ʾ
        /*
        std::cout << "��������ʱ�䣺 " << Missile.gettime() << endl;
        std::cout << "����������̣� " << Missile.getx_ned() << endl;
        std::cout << "�������ո߶ȣ� " << Missile.gety_ned() << endl;
        std::cout << "���������ٶȣ� " << Missile.getV() << endl;
        std::cout << "�Ѱ�����" << R << endl;
        */
    }

    std::cout << "Step 5 : ������ϣ��˶������ѱ��棬��鿴��Ӧ�ļ���" << endl << endl;
    std::cout << "�Ѱ�����" << R << endl;
    std::cout << "��������ʱ�䣺 " << Missile.gettime() << endl;
    std::cout << "����������̣� " << Missile.getx_ned() << endl;
    std::cout << "�������ո߶ȣ� " << Missile.gety_ned() << endl;
    std::cout << "���������ٶȣ� " << Missile.getV() << endl;

    std::system("pause");
    return 0;
};