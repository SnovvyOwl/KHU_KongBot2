% THIS CODE IS Written by SeongWon Lee 
clc;clear;clf;
angle=-xlsread("angle.csv");
vel=-xlsread("vel.csv");
time=xlsread("time.csv");
pos=-xlsread("pos.csv");
subplot(3,1,1)
plot(time,angle)
subplot(3,1,2)
plot(time,vel)
subplot(3,1,3)
plot(time,pos)