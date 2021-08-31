#include <server.h>
#define BUFF_SIZE 8
using namespace std;

Server::Server(const char *_ip, int _port)
{
    ip = (char *)_ip;
    server = socket(AF_INET, SOCK_STREAM, 0);//서버소켓 생성
    if (server == -1)
    {
        cerr << "\n Socket creation error \n";//소켓 생성 에러
        //sock_receive="Quit";
        exit(1);
    }
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    startServer();//서버 시작
}
void Server::startServer()
{
    cout << "[Start Server]\n";
    cout << "Server ip -> " << ip << endl;
    cout << "Server port -> " << port << endl;
    if (bind(server, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)//소켓 서버 바인딩
    {
        cerr << "Bind ERROR" << endl;//연결이 부정확할 경우 종료
        //sock_receive="Quit";
        exit(1);
    }
    if (listen(server, 1) < 0)
    {
        cerr << "Listen ERROR" << endl;// 소켓연결을 할수 있도록 열음 
        //sock_receive="Quit";
        exit(1);
    }
    cout << "Wait...\n";
    client_addr_len = sizeof(client_addr);
    client = accept(server, (struct sockaddr *)&client_addr, &client_addr_len);//클라이언트가 연결될때까지 기다힘
    printf("Connection from: %s\n", inet_ntoa(client_addr.sin_addr));
    char buffer[BUFF_SIZE] = {0};
    runServer();//서버 시작
}
void Server::runServer()
{
    thread inputCMD([&](){ keyboardInput(); });//키보드 입력쓰레드
    thread sockReceive([&](){ receiving(); });//클라이언트에서 받는 쓰레드
    thread sockSend([&](){ sending(); });//클라이언트로 보내는 쓰레드
    //각자 메인쓰레드와 따로 놈
    sockReceive.detach();
    inputCMD.detach();
    sockSend.detach();
    do
    {
        cout << msgReceive;//받은 메세지 출력
        switch (int(CMD))
        {
        // 키보드 입력에 따라 다르게 출력되는 값을 보여주기 위해서 만든 스위치 케이스문이다.
        case 119:
            //CMD=w
            cout << "go\n";

            break;

        case 115:
            //CMD=s
            cout << "back\n";

            break;

        case 87:
            //CMD=W
            cout << "GO\n";

            break;

        case 83:
            //CMD=S
            cout << "BACK\n";

            break;

        case 97:
            //CMD=a
            cout << "chage roll - direction \n";

            break;

        case 100:
            //CMD=d
            cout << "chage roll + direction \n";

            break;

        case 106:
            //CMD=j
            cout << "chage yaw  +15 degree direction \n";
            break;

        case 107:
            //CMD=k
            cout << "chage yaw  -15 degree direction \n";
            break;

        case 113:
            cout<<"stop"<<endl;
            break;

        default:
            //cout<< "Wrong OR EMPTY CMD\n";
            break;
        }

    } while (CMD != 'q');//키보드에서 입력값이 q일경우 서버를 종료한다.
    stopServer();
}

void Server::receiving()//소켓에서 데이터를 받는 함수 
{
    char buffer[BUFF_SIZE] = {0};
    // stringstream ss;
    do
    {
        read(client, buffer, BUFF_SIZE);
        msgReceive = buffer;
        if (msgReceive.size())
        {
            buffer[0] = {0,};
        }
    } while (CMD != 'q');
    exit(1);
}

void Server::sending()//서버에서 보내는 함수
{
    do
    {
        //penR,penL,IDU,tilt
        msgSend = to_string(CMD)+"\n";
        send(client, msgSend.c_str(), msgSend.size(), 0);
    } while (CMD != 'q');
    exit(1);
}

void Server::keyboardInput()//키보드 인풋에 대한 함수 q를 입력받으면 종료
{
    //KEBOARD INPUT
    do
    {
        cin >> CMD;
    } while (CMD != 'q');
    exit(1);
}
void Server::stopServer()//종료 함수
{
    msgSend = "q";
    send(client, msgSend.c_str(), msgSend.size(), 0);
    cout << "[stop server]\n";
    close(client);
    close(server);
    cout << "OFF";
    exit(1);
}
