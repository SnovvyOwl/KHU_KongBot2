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
        cerr<< "\n Socket creation error \n";//소켓이 만들어지지 않았을 경우 프로그램 종료
        msgReceive="quit";
	    exit(1);
    }
    if (connect(client, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0){    
        cerr<<"\nConnection Failed \n"; //서버와 연결되지 않았을 경우 프로그램 종료
        msgReceive="quit";
        exit(1);
    }
    runClient();//runClient함수 호출
}
       
void Client::runClient(){
    thread sockReceive([&](){receive_send();});//소켓통신을 통해 데이터만 주고 받는 쓰레드
    int i=0;
    sockReceive.detach();//메인 쓰레드와 생성된 쓰레드가 완전히 독립적으로 작동함.
    do{ 
        msgSend=to_string(i)+"\n";
        send(client,msgSend.c_str(),msgSend.size(),0); //서버로 i를 보냄
        cout<<msgReceive<<endl;//서버에서 보낸 메세지 출력
    } while (msgReceive!= "q");//서버에서 받은 데이터가 q로 시작될경우 종료
    close(client); //보통 서버가 먼저 꺼지면 프로그램이 자동 종료 된다.
}

void Client::receive_send(){
    //Socket receive[THREAD 2]
    char buffer[BUFF_SIZE]={0};//서버에서 보낸 데이터를 받을 버퍼
    do {
        read(client,buffer,BUFF_SIZE);//받은 데이터를 버퍼에 저장
        msgReceive=buffer;//버퍼를 string으로 변환
        buffer[0]={0,};//버퍼 초기화
    } while ( msgReceive != "q");
}