#pragma once
#include<random>
#include<sstream>
#include<iostream>
#include<stdio.h>
#include<string>
#include<unistd.h>
#include<arpa/inet.h>
#include<sys/socket.h>
#include<thread>
#include<netdb.h>
#include<fstream>
#include<math.h>
#include<state.h>
#define BUFF_SIZE 64
using namespace std;

normal_distribution<double> dist_W(0,1.0);
normal_distribution<double> dist_V(0,1.0);
class Server{
	private:
		char *ip;
		int port=0;
		int server=0;
		int client=0;
		struct sockaddr_in client_addr;
		struct sockaddr_in server_addr;
		char CMD;
		socklen_t client_addr_len=sizeof(client_addr);
        string msgReceive="";
        string msgSend="";
        Pendulum pen;
        IDU idu;
        Shell shell;
        Tilt tilt;
        float roll=0;
        float pitch=0;
        float yaw=0;
        float encoder=0;
        float tiltTheta=0;
        int penLM=0;
        int penRM=0;
        int iduM=0;
        int tiltM=0;
        float desireVel=0;
        float desireRoll=0;
        float desireYaw=0;
        int sock_recv=0;
        ofstream fout;
    public:
        Server(const char *_ip,int _port){
            ip=(char*)_ip;
            port=_port;
            server=socket(AF_INET,SOCK_STREAM,0);
            if (server==-1){
                cerr<<"\n Socket creation error \n";
                //sock_receive="Quit";
                exit(1);
            }
            server_addr.sin_family=AF_INET;
            server_addr.sin_port=htons(port);
            server_addr.sin_addr.s_addr=htonl(INADDR_ANY);
            startServer();
        }
        void startServer(){
            cout<<"[Start Server]\n";
            cout<<"Server ip -> "<<ip<<endl;
            cout<<"Server port -> "<<port<<endl;
            if(bind(server,(struct sockaddr*)&server_addr,sizeof(server_addr))<0){
                cerr<<"Bind ERROR"<<endl;
                //sock_receive="Quit";
                exit(1);
            }
            if (listen(server,1)<0){
                cerr<<"Listen ERROR"<<endl;
                //sock_receive="Quit";
                exit(1);
            }
            cout<<"Wait...\n";
            client_addr_len=sizeof(client_addr);
            client=accept(server, (struct sockaddr *)&client_addr,&client_addr_len);
            printf("Connection from: %s\n",inet_ntoa(client_addr.sin_addr));   
            char buffer[BUFF_SIZE]={0};
            read(client,buffer,BUFF_SIZE);
            msgReceive=buffer;
            cout<<msgReceive<<endl;
            runServer();
        }
        void runServer(){
            fout.open("sensor.txt");
            tilt.setDist(dist_W,dist_V);
            shell.setDist(dist_W,dist_V);
            cout<<"Starting robot\n";
            thread inputCMD([&](){keyboardInput();});
            thread sock([&](){recv_send();});
            thread stable([&](){iduStable();});
            inputCMD.detach();
            sock.detach();
            stable.detach();
            do{ 
                cout<<msgSend<<endl;
                if(sock_recv){
                    fout<<roll<<","<<pitch<<","<<yaw<<","<<encoder<<","<<tiltTheta<<endl;
                    sock_recv=0;
            
                }
                shell.calAngularVelocity(pitch,encoder,pen.getTheta(),pen.getVel());
                tilt.calRollAngle(roll);
                switch (int(CMD)){
                    
                    //CMD to NANO
                    // * IDU MOTOR INPUT , PENDULUM RIGHT MOTOR INPUT , PENDDULUM LEFT MOTOR INPUT, CONTROLL ROLL MOTOR INPUT
                    //string CMD="* 1500 1500 1500 1500\n"; //fake CMD
                    //serialPuts(NanoCMD,CMD.c_str());
                    // keyboard INPUT
                    // w (foward_1) 
                    // s (backward_1
                    // a (chage roll) - direction  
                    // d (chage roll) + direction 
                    // j (change yaw) +
                    // k (chage yaw ) -
                    // W (Foward_2)]
                    // S (Backward_2)
                    case 119 :
                        //CMD=w
                        cout<< "go\n";
                        desireVel=1;
                        penLM=pen.motor(shell.speedControl(desireVel));
                        penLM=penRM;
                        break;

                    case 115 :
                        //CMD=s
                        cout<<"back\n";
                        desireVel=-1;
                        penLM=pen.motor(shell.speedControl(desireVel));
                        penLM=penRM;
                        break;

                    case 87:
                        //CMD=W
                        cout<< "GO\n";
                        desireVel=2;
                        penLM=pen.motor(shell.speedControl(desireVel));
                        penLM=penRM;
                        break;
            
                    case 83:
                        //CMD=S
                        cout<< "BACK\n";
                        desireVel=-2;
                        penLM=pen.motor(shell.speedControl(desireVel));
                        penLM=penRM;
                        break;
            
                    case 97:
                        //CMD=a
                        cout<< "chage roll - direction \n";
                        desireRoll=-1;
                        tiltM=tilt.rollControl(desireRoll);
                        break;

                    case 100:
                        //CMD=d
                        cout<< "chage roll + direction \n";
                        desireRoll=1;
                        tiltM=tilt.rollControl(desireRoll);
                        break;

                    // case 106:
                    //     //CMD=j
                    //     cout<< "chage yaw  +15 degree direction \n";
                    //     penLM=1800;
                    //     penRM=1200;
                    //     break;

                    // case 107:
                    //     //CMD=k
                    //     cout<< "chage yaw  -15 degree direction \n";
                    //     penLM=1200;
                    //     penRM=1800;
                    //     break;

                    case 113:
                        msgSend="* 1500 1500 1500 1500\n";//stop Motor
                        send(client,msgSend.c_str(),msgSend.size(),0); 
                        break;
                
                    default:
                        //cout<< "Wrong OR EMPTY CMD\n";
                        break;
                }

            } while (CMD != 'q');
            stopServer();
        }
        void recv_send(){
            char buffer[BUFF_SIZE]={0};
            do{
                read(client,buffer,BUFF_SIZE);
                msgReceive=buffer;
                if(msgReceive.size()){
                    msgReceive=msgReceive.substr(1,msgReceive.find("\n")-1);
                    replace(msgReceive.begin(), msgReceive.end(), ',', ' ');
                    stringstream ss(msgReceive);
                    ss>>pitch;
                    ss>>roll;
                    ss>>yaw;
                    ss>>encoder;
                    ss>>tiltTheta;
                    tilt.setTilt(tiltTheta);
                    buffer[0]={0,};
                    pitchNorm();
                    ss.clear();
                    sock_recv=1;
                }
                msgSend="* "+to_string(penLM)+" "+to_string(penRM)+" "+to_string(iduM)+" "+to_string(tiltM)+"\n";
                send(client,msgSend.c_str(),msgSend.size(),0); 
            }while(CMD !='q');
            exit(1);
        }

        void keyboardInput() {
            //KEBOARD INPUT
            do {
                cin >> CMD;
            } while (CMD != 'q');
            exit(1);
        }
        void pitchNorm(){
            if (pitch>=0){
                pitch=-(pitch-180);
                return;
            }
            else if (pitch<0){
                pitch=-(pitch+180);
                return;
            }
            else if ((pitch==180)||pitch==-180){
                pitch=0;
            }
        }
        void iduStable(){
            do {
                //pitchNorm();
                iduM=idu.stableControl(pitch);
            } while (CMD != 'q');
            exit(1);
        }
        void stopServer(){
            msgSend="q";
            fout.close();
            send(client,msgSend.c_str(),msgSend.size(),0); 
            cout<<"[stop server]\n";
            close(client);
            close(server);
            cout<<"OFF";
            exit(1);
        }
};