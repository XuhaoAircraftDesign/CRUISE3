#include "missile.h"   // 巡飞弹
#include "Target.h"    // 地面目标   
#include <iostream>
using namespace std;

// 主函数 
int main()
{
    std::cout << "Step 1 : 初始化目标运动参数！" << endl;
    double Vt0 = 10;        
    double thetat0 = 0;    
    double psivt0 = 0;         
    double xt0 = 1000;          
    double yt0 = 0;              
    double zt0 = 300;          
    double tt0 = 0;
    /*
    std::cout << "请输入目标弹道倾角(单位：度/deg)： " << endl;
    std::cin >> thetat0;
    std::cout << "请输入目标弹道偏角(单位：度/deg)： " << endl;
    std::cin >> psivt0;
    std::cout << "请输入目标水平位置(单位：m)： " << endl;
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

    std::cout << "Step 2 : 初始化导弹运动参数！" << endl;
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

    std::cout << "Step 3 : 初始化导弹和目标之间距离！" << endl;
    double xr0 = x0_target[3] - x0_missile[3];
    double yr0 = x0_target[4] - x0_missile[4];
    double zr0 = x0_target[5] - x0_missile[5];
    double R = sqrt(xr0 * xr0 + yr0 * yr0 + zr0 * zr0);     // 弹目距离
    Missile.setrange(R);              

    std::cout << "Step 4 : 初始化完毕，开始计算！" << endl;
    while (Missile.gety_ned() >= 0 && R >= 1)
    {
        // 目标仿真一步长
        Target.OneStep(7, 0.01);

        // 导弹仿真一步长
        Missile.OneStep(7, 0.01, Target.getut_ned(), Target.getvt_ned(), Target.getwt_ned(), 
                        Target.getxt_ned(), Target.getyt_ned(), Target.getzt_ned());

        // 下一步长弹目距离
        R = Missile.getrange();

        // 终端显示
        /*
        std::cout << "导弹飞行时间： " << Missile.gettime() << endl;
        std::cout << "导弹最终射程： " << Missile.getx_ned() << endl;
        std::cout << "导弹最终高度： " << Missile.gety_ned() << endl;
        std::cout << "导弹最终速度： " << Missile.getV() << endl;
        std::cout << "脱靶量：" << R << endl;
        */
    }

    std::cout << "Step 5 : 计算完毕，运动数据已保存，请查看相应文件！" << endl << endl;
    std::cout << "脱靶量：" << R << endl;
    std::cout << "导弹飞行时间： " << Missile.gettime() << endl;
    std::cout << "导弹最终射程： " << Missile.getx_ned() << endl;
    std::cout << "导弹最终高度： " << Missile.gety_ned() << endl;
    std::cout << "导弹最终速度： " << Missile.getV() << endl;

    std::system("pause");
    return 0;
};