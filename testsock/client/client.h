#pragma once
#include<iostream>
#include<string.h>
#include<wiringPi.h>
#include<wiringSerial.h>
#include<sstream>
#include<thread>
#include<unistd.h>
#include<arpa/inet.h>
#include<sys/socket.h>
#include<netdb.h>
#include<raspicam/raspicam_cv.h>
#include<opencv2/imgproc.hpp>
#include<opencv2/opencv.hpp>
#include<math.h>
using namespace std;
using namespace cv;
#define BUFF_SIZE 8
class Client{
    private:
        int client=0;
        
        struct sockaddr_in server_addr;
        string msgReceive="";
        string msgSend="";
        struct hostent *he;
        string CMD="";

    public:
        Client(const char *hostname, const int _port)
        {   
            he=gethostbyname(hostname); // server url
            client = socket( AF_INET, SOCK_STREAM, 0);
            server_addr.sin_family = AF_INET;
            server_addr.sin_port = htons(_port);
            server_addr.sin_addr.s_addr=*(long*)(he->h_addr_list[0]);
            if(client==-1){
                cerr<< "\n Socket creation error \n";
                msgReceive="quit";
		        exit(1);
            }
            if (connect(client, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0){    
                cerr<<"\nConnection Failed \n"; 
                msgReceive="quit";
		        exit(1);
            }
            runClient();
        }
       
        void runClient(){
            thread sockReceive([&](){receive_send();});
            int i=0;
            sockReceive.detach();
            do{ 
                msgSend=to_string(i)+"\n";
                send(client,msgSend.c_str(),msgSend.size(),0); 
                i++;
            } while (CMD[0] != 'q');
            //cout << "quit" << endl;  
            quitClient();
        }
        void quitClient(){
            cout<<"Close.."<<endl;

            close(client);
            cout<<"OFF"<<endl;
            exit(1);

        }
        void receive_send(){
            //Socket INPUT and SEND Nano [THREAD 3]
            char buffer[BUFF_SIZE]={0};
            do {
                read(client,buffer,BUFF_SIZE);
                CMD=buffer;
                //serialPuts(Nano,CMD.c_str());
                buffer[0]={0,};
            } while (CMD != "q");
        }
};
