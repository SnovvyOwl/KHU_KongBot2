#pragma once
#include<iostream>
#include<sys/socket.h>
#include<unistd.h>
#include<stdio.h>
#include<sstream>
#include<state.h>
#include<arpa/inet.h>
#include<thread>
#include<string>
#include<netdb.h>
#define BUFF_SIZE 8
using namespace std;
class Server{
    private:
        char *ip;
        int port=13000;
        int server=0;
        int client=0;
        struct sockaddr_in client_addr;
        struct sockaddr_in server_addr;
        socklen_t client_addr_len=sizeof(client_addr);
        string sock_receive="";
        string sock_send="";
    public:
        Server(const char *_ip,int _port){
            ip=(char*)_ip;
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
            if(bind(server,(struct sockaddr*)&server_addr,sizeof(server_addr)<0)){
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
            client=accept(server,)
        }


};