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
    char *ip;
    int port = 13000;
    int server = 0;
    int client = 0;
    struct sockaddr_in client_addr;
    struct sockaddr_in server_addr;
    char CMD;
    socklen_t client_addr_len = sizeof(client_addr);
    string msgReceive = "";
    string msgSend = "";

public:
    Server(const char *_ip, int _port);
    void startServer();
    void runServer();
    void receiving();
    void sending();
    void keyboardInput();
    void stopServer();
};