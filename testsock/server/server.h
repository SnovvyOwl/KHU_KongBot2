#pragma once
#include <iostream>
#include <stdio.h>
#include <string>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <thread>
#include <netdb.h>
#define BUFF_SIZE 8
using namespace std;

class Server
{
private:
    char *ip;//서버 IP주소
    int port = 13000;//포트번호
    int server = 0;//서버 소켓
    int client = 0;//클라이언트 소켓
    struct sockaddr_in client_addr;//클라이언트 소켓 주소  구조체
    struct sockaddr_in server_addr;//서버 소켓 주소 구조체
    char CMD;//키보드 입력 명령어
    socklen_t client_addr_len = sizeof(client_addr);//소켓주소 길이
    string msgReceive = "";//클라이언트에서 보낸 데이터
    string msgSend = "";//클라이언트로 보낼 데이터

public:
    Server(const char *_ip, int _port);//생성자
    void startServer();//서버시작 메서드
    void runServer();//서버 동작중 메서드
    void receiving();//서버에서 데이터 보내는 메서드
    void sending();//클라이언트에서 데이터 받는 메서드
    void keyboardInput();//키보드에서 입력 받는 함수
    void stopServer();// 서버 종료 메서드
};