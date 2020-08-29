% THIS CODE IS Written by SeongWon Lee 
% THIS CODE IS based Practical Modeling and System Identification of R/C
% Servo Motors( 18th IEEE International Conference on Control Applications
% Part of 2009 IEEE Multi-conference on Systems and Control
% Saint Petersburg, Russia, July 8-10, 2009)
clc;clear;clf;
angle=-xlsread("angle.csv");
time=xlsread("time.csv");
y=[(time(:,1)-2624), angle];
y=unique(y,'rows','stable'); % REFINED DATA;

%step function
u=[y(:,1),zeros(21235,1)];
%u=unique(u,"rows","stable");
t1=find(u(:,1)==1009);
t2=find(u(:,1)==3019);
u(t1:t2,2)=8.3;
t1=find(u(:,1)==5006);
t2=find(u(:,1)==7009);
u(t1:t2,2)=8.3;
t1=find(u(:,1)==9008);
t2=find(u(:,1)==11005);
u(t1:t2,2)=16.775;
t1=find(u(:,1)==12991);
t2=find(u(:,1)==15009);
u(t1:t2,2)=16.775;
t1=find(u(:,1)==16998);
t2=find(u(:,1)==19000);
u(t1:t2,2)=16.775;
plot(y(:,1),y(:,2),u(:,1),u(:,2))