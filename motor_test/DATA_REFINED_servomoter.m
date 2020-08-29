% THIS CODE IS Written by SeongWon Lee 
% THIS CODE IS based Practical Modeling and System Identification of R/C
% Servo Motors( 18th IEEE International Conference on Control Applications
% Part of 2009 IEEE Multi-conference on Systems and Control
% Saint Petersburg, Russia, July 8-10, 2009)
clc;clear;clf;
angle=-xlsread("angle.csv");
time=xlsread("time.csv");
y=[(time(:,1)-2624)./1000, angle];
y=unique(y,'rows','stable'); % REFINED DATA;

%step function
u=[y(:,1),zeros(21235,1)];
%u=unique(u,"rows","stable");
t1=find(u(:,1)==1.009);
t2=find(u(:,1)==3.019);
u(t1:t2,2)=8.3;
t1=find(u(:,1)==5.006);
t2=find(u(:,1)==7.009);
u(t1:t2,2)=8.3;
t1=find(u(:,1)==9.008);
t2=find(u(:,1)==11.005);
u(t1:t2,2)=16.775;
t1=find(u(:,1)==12.991);
t2=find(u(:,1)==15.009);
u(t1:t2,2)=16.775;
t1=find(u(:,1)==16.998);
t2=find(u(:,1)==19.000);
u(t1:t2,2)=16.775;
plot(y(:,1),y(:,2),u(:,1),u(:,2))
clear angle t1 t2 time
dlmwrite("RefinedY.txt",y)
dlmwrite("RefinedU.txt",u)
