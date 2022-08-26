function [ ] = plot3_project( X,Y,Z,m_freq)
%PLOT3_PROJECT Summary of this function goes here
%   Detailed explanation goes here
hold on
[m,n]=size(X);
plot3(X,Z,zeros(m,1),'k')
m_m = floor(m/m_freq);
plot3([X(1,1) X(1,1)],[Z(1,1) Z(1,1)],[Y(1,1) 0],'--k');
for i = 1:1:m_freq
    plot3([X(m_m*i,1) X(m_m*i,1)],[Z(m_m*i,1) Z(m_m*i,1)],[Y(m_m*i,1) 0],'--k');
end
end

