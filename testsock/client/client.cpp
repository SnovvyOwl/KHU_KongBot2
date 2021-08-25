#include<client.h>
using namespace std;
#define BUFF_SIZE 8
Client::Client(const char *hostname, const int _port)
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
       
void Client::runClient(){
    thread sockReceive([&](){receive_send();});
    int i=0;
    sockReceive.detach();
    do{ 
        msgSend=to_string(i)+"\n";
        send(client,msgSend.c_str(),msgSend.size(),0); 
        i++;
    } while (msgReceive[0] != 'q');
    close(client);
}
void Client::receive_send(){
    //Socket receive[THREAD 2]
    char buffer[BUFF_SIZE]={0};
    do {
        read(client,buffer,BUFF_SIZE);
        msgReceive=buffer;
        //serialPuts(Nano,CMD.c_str());
        buffer[0]={0,};
    } while ( msgReceive != "q");
}