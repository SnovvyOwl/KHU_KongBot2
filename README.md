# KHU_KongBot 2
\
\
[About]\
In 2019 Spring semester, We made spherical Robot,"KHU_KongBot1". But it had many Problem.\
Therefore, We redesign KHU_KongBot1. and It called KHU_KongBot2.\
\
\
2019년 1학기 '기계공학종합설계-로봇공학연구실 2조' 에서 태어난 말썽꾸러기 KHU_KongBot 1을 개조하여 
말 잘듣는 KHU_KongBot 2로 새로 태어나게 만드는 팀입니다.
\
\
[Team Member]

Seongwon Lee, Myenghyen Kim, Hyungju Choi\
\
\
[HardWare]\
RaspberryPi 4 4GB(Raspbian Buster){Testing Hardware is Raspberry Pi 3B / Raspbian Buster }\
Raspberry Camera V 2.1\
E2BOX EBIMU 9DOF V4\
Arduino Nano\
SPT5325LV-360 Servo\
TowerPro MG 946R 3ea\
Autonomics E50S8-3600-3-T-5\
\
\
[Dependency]\
OpenCV 3.4.4 required\
Raspicam 0.1.6 required\
Thread LIBS\
wiringPi LIBS\
\
\
[How To Build]\
cd ~/KHU_KongBot2\
mkdir build\
cd build\
cmake ..\
make\
\
\
[How to Run]\
cd ~/KHU_KongBot2/build\
./main
