#include <server.h>
#define BUFF_SIZE 8
using namespace std;

Server::Server(const char *_ip, int _port)
{
    ip = (char *)_ip;
    server = socket(AF_INET, SOCK_STREAM, 0);
    if (server == -1)
    {
        cerr << "\n Socket creation error \n";
        //sock_receive="Quit";
        exit(1);
    }
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    startServer();
}
void Server::startServer()
{
    cout << "[Start Server]\n";
    cout << "Server ip -> " << ip << endl;
    cout << "Server port -> " << port << endl;
    if (bind(server, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
    {
        cerr << "Bind ERROR" << endl;
        //sock_receive="Quit";
        exit(1);
    }
    if (listen(server, 1) < 0)
    {
        cerr << "Listen ERROR" << endl;
        //sock_receive="Quit";
        exit(1);
    }
    cout << "Wait...\n";
    client_addr_len = sizeof(client_addr);
    client = accept(server, (struct sockaddr *)&client_addr, &client_addr_len);
    printf("Connection from: %s\n", inet_ntoa(client_addr.sin_addr));
    char buffer[BUFF_SIZE] = {0};
    runServer();
}
void Server::runServer()
{
    thread inputCMD([&](){ keyboardInput(); });
    thread sockReceive([&](){ receiving(); });
    thread sockSend([&](){ sending(); });

    sockReceive.detach();
    inputCMD.detach();
    sockSend.detach();
    do
    {
        cout << msgReceive;
        switch (int(CMD))
        {
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
            msgSend = "*1500,1500,1500,1500\n"; //stop Motor
            send(client, msgSend.c_str(), msgSend.size(), 0);
            break;

        default:
            //cout<< "Wrong OR EMPTY CMD\n";
            break;
        }

    } while (CMD != 'q');
    stopServer();
}

void Server::receiving()
{
    char buffer[BUFF_SIZE] = {0};
    // stringstream ss;
    do
    {
        read(client, buffer, BUFF_SIZE);
        msgReceive = buffer;
        if (msgReceive.size())
        {

            buffer[0] = {
                0,
            };
        }
    } while (CMD != 'q');
    exit(1);
}

void Server::sending()
{
    int i = 0;
    do
    {
        //penR,penL,IDU,tilt
        msgSend = "*" + to_string(i) + " " + to_string(i) + " " + to_string(i) + " " + to_string(i) + "\n";
        send(client, msgSend.c_str(), msgSend.size(), 0);
    } while (CMD != 'q');
    exit(1);
}

void Server::keyboardInput()
{
    //KEBOARD INPUT
    do
    {
        cin >> CMD;
    } while (CMD != 'q');
    exit(1);
}
void Server::stopServer()
{
    msgSend = "q";
    send(client, msgSend.c_str(), msgSend.size(), 0);
    cout << "[stop server]\n";
    close(client);
    close(server);
    cout << "OFF";
    exit(1);
}
