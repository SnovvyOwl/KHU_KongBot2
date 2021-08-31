#pragma once
#include<iostream>
#include<string.h>
#include<thread>
#include<unistd.h>
#include<arpa/inet.h>
#include<sys/socket.h>
#include<netdb.h>
using namespace std;
#define BUFF_SIZE 8
class Client{
    private:
        int client=0; 
        struct sockaddr_in server_addr; //서버의 주소가 저장되는 구조체
        string msgReceive="";//받은 메세지 저장
        string msgSend="";//보낸 메세지
        struct hostent *he;//server가 URL주소일 경우 일차적으로 hostent 구조체의 이름으로 넣어짐.
    public:
        Client(const char *hostname, const int _port);//client 생성자 prameter로 서버주소와 포트를 넣어줌
        void runClient();//client시작 
        void receive_send();//서버와 주고 받는 쓰레드
};