%% ���� C++ ����Ŀ����ԡ�
clear;
clc;
r2d = 180/pi;

%% ����C++����
Missile = dlmread('missile.txt');
Target = dlmread('Target.txt');
Guidance = dlmread('guidance.txt');
Engine = dlmread('engine.txt');
nT = length(Target(:,1));
nM = length(Missile(:,1));

%% ���ǲ໬�Ƕ�ƫ��
figure;
subplot(3,1,1);
plot(Missile(:,1),Missile(:,3)*r2d,'linewidth',2);
legend('C++');
title('����');
xlabel('ʱ��/s');
grid on;
subplot(3,1,2);
plot(Missile(:,1),Missile(:,4)*r2d,'linewidth',2);
legend('C++');
title('�໬��');
xlabel('ʱ��/s');
grid on;
subplot(3,1,3);
plot(Missile(:,1),Missile(:,10)*r2d,'linewidth',2);
legend('C++');
title('��ƫ��');
xlabel('ʱ��/s');
grid on;

%% ������� ������ƫ�ǡ��ٶȹ�ת��
figure;
subplot(3,1,1);
plot(Missile(:,1),Missile(:,5)*r2d,'linewidth',2);
legend('C++');
title('�������');
xlabel('ʱ��/s');
grid on;
subplot(3,1,2);
plot(Missile(:,1),Missile(:,6)*r2d,'linewidth',2);
legend('C++');
title('����ƫ��');
xlabel('ʱ��/s');
grid on;
subplot(3,1,3);
plot(Missile(:,1),Missile(:,11)*r2d,'linewidth',2);
legend('C++');
title('�ٶȹ�ת��');
xlabel('ʱ��/s');
grid on;

%% ת������
figure;
subplot(3,1,1);
plot(Engine(:,1),Engine(:,4),'linewidth',2);
legend('C++');
title('Jx');
xlabel('ʱ��/s');
grid on;
subplot(3,1,2);
plot(Engine(:,1),Engine(:,5),'linewidth',2);
legend('C++');
title('Jy');
xlabel('ʱ��/s');
grid on;
subplot(3,1,3);
plot(Engine(:,1),Engine(:,6),'linewidth',2);
legend('C++');
title('Jz');
xlabel('ʱ��/s');
grid on;

%% ����������
figure;
subplot(2,1,1);
plot(Engine(:,1),Engine(:,2),'linewidth',2);
legend('C++');
title('mass');
xlabel('ʱ��/s');
grid on;
subplot(2,1,2);
plot(Engine(:,1),Engine(:,3),'linewidth',2);
legend('C++');
title('thrust');
xlabel('ʱ��/s');
grid on;

%% �ٶ�
figure;
plot(Missile(:,1),Missile(:,2),'linewidth',2);
xlabel('ʱ��/s');
legend('C++');
title('�ٶ�');
grid on;

%% ������άǦ����켣����Ŀ����
figure;
subplot(2,1,1);
plot(Missile(:,7),Missile(:,8),'linewidth',2);
hold on;
plot(Target(:,5),Target(:,6),'linewidth',2);
legend('missile', 'target');
xlabel('���');
ylabel('�߶�');
title('Ǧ����켣');
grid on;
subplot(2,1,2);
plot(Guidance(:,1),Guidance(:,4),'linewidth',2);
legend('��Ŀ����');
xlabel('ʱ��/s');
ylabel('��Ŀ����');
title('��Ŀ����');
grid on;

%% ��Ŀ��ά�ռ�켣
figure;
plot3(Missile(:,7),Missile(:,9),Missile(:,8),'linewidth',2);
hold on;
plot3(Target(:,5),Target(:,7),Target(:,6),'linewidth',2);
xlabel('���');
ylabel('����λ��');
zlabel('�߶�');
title('�ռ�켣');
grid on;

%% ���ظ���
figure;
subplot(2,1,1); 
plot(Guidance(:,1),Guidance(:,2)/9.8,'linewidth',2);
legend('nyc');
title('nyc');
xlabel('ʱ��/s');
grid on;
subplot(2,1,2);
plot(Guidance(:,1),Guidance(:,3)/9.8,'linewidth',2);
legend('nzc');
title('nzc');
xlabel('ʱ��/s');
grid on;