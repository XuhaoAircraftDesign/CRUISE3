%% 测试 C++ 程序的可行性。
clear;
clc;
r2d = 180/pi;

%% 读入C++数据
Missile = dlmread('missile.txt');
Target = dlmread('Target.txt');
Guidance = dlmread('guidance.txt');
Engine = dlmread('engine.txt');
nT = length(Target(:,1));
nM = length(Missile(:,1));

%% 攻角侧滑角舵偏角
figure;
subplot(3,1,1);
plot(Missile(:,1),Missile(:,3)*r2d,'linewidth',2);
legend('C++');
title('攻角');
xlabel('时间/s');
grid on;
subplot(3,1,2);
plot(Missile(:,1),Missile(:,4)*r2d,'linewidth',2);
legend('C++');
title('侧滑角');
xlabel('时间/s');
grid on;
subplot(3,1,3);
plot(Missile(:,1),Missile(:,10)*r2d,'linewidth',2);
legend('C++');
title('舵偏角');
xlabel('时间/s');
grid on;

%% 弹道倾角 、弹道偏角、速度滚转角
figure;
subplot(3,1,1);
plot(Missile(:,1),Missile(:,5)*r2d,'linewidth',2);
legend('C++');
title('弹道倾角');
xlabel('时间/s');
grid on;
subplot(3,1,2);
plot(Missile(:,1),Missile(:,6)*r2d,'linewidth',2);
legend('C++');
title('弹道偏角');
xlabel('时间/s');
grid on;
subplot(3,1,3);
plot(Missile(:,1),Missile(:,11)*r2d,'linewidth',2);
legend('C++');
title('速度滚转角');
xlabel('时间/s');
grid on;

%% 转动惯量
figure;
subplot(3,1,1);
plot(Engine(:,1),Engine(:,4),'linewidth',2);
legend('C++');
title('Jx');
xlabel('时间/s');
grid on;
subplot(3,1,2);
plot(Engine(:,1),Engine(:,5),'linewidth',2);
legend('C++');
title('Jy');
xlabel('时间/s');
grid on;
subplot(3,1,3);
plot(Engine(:,1),Engine(:,6),'linewidth',2);
legend('C++');
title('Jz');
xlabel('时间/s');
grid on;

%% 质量、推力
figure;
subplot(2,1,1);
plot(Engine(:,1),Engine(:,2),'linewidth',2);
legend('C++');
title('mass');
xlabel('时间/s');
grid on;
subplot(2,1,2);
plot(Engine(:,1),Engine(:,3),'linewidth',2);
legend('C++');
title('thrust');
xlabel('时间/s');
grid on;

%% 速度
figure;
plot(Missile(:,1),Missile(:,2),'linewidth',2);
xlabel('时间/s');
legend('C++');
title('速度');
grid on;

%% 导弹二维铅垂面轨迹、弹目距离
figure;
subplot(2,1,1);
plot(Missile(:,7),Missile(:,8),'linewidth',2);
hold on;
plot(Target(:,5),Target(:,6),'linewidth',2);
legend('missile', 'target');
xlabel('射程');
ylabel('高度');
title('铅垂面轨迹');
grid on;
subplot(2,1,2);
plot(Guidance(:,1),Guidance(:,4),'linewidth',2);
legend('弹目距离');
xlabel('时间/s');
ylabel('弹目距离');
title('弹目距离');
grid on;

%% 弹目三维空间轨迹
figure;
plot3(Missile(:,7),Missile(:,9),Missile(:,8),'linewidth',2);
hold on;
plot3(Target(:,5),Target(:,7),Target(:,6),'linewidth',2);
xlabel('射程');
ylabel('侧向位置');
zlabel('高度');
title('空间轨迹');
grid on;

%% 过载跟踪
figure;
subplot(2,1,1); 
plot(Guidance(:,1),Guidance(:,2)/9.8,'linewidth',2);
legend('nyc');
title('nyc');
xlabel('时间/s');
grid on;
subplot(2,1,2);
plot(Guidance(:,1),Guidance(:,3)/9.8,'linewidth',2);
legend('nzc');
title('nzc');
xlabel('时间/s');
grid on;