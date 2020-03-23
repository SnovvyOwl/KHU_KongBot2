#include<thread>
#include<iostream>
using namespace std;
void input(char &CMD) {
    do {
        cin >> CMD;
    } while (CMD != 'q');
    
}

int main()
{
    char CMD;
    thread inputCMD(&input, ref(CMD));//INPUT command Thread.....
    inputCMD.detach();
    do{
        switch (int(CMD)){
            case 119 :
                //CMD=w
                cout<< "go\n";
                break;
            case 115 :
                //CMD=s
                cout<<"back\n";
                break;

            case 87:
                //CMD=W
                cout<< "GO\n"<<endl;
                break;
            
            case 83:
                //CMD=S
                cout<< "BACK\n"<<endl;
                break;
            
            case 97:
                //CMD=a
                cout<< "chage roll - direction \n"<<endl;
                break;
            case 100:
                //CMD=d
                cout<< "chage roll + direction \n"<<endl;
                break;
            case 106:
                //CMD=j
                cout<< "chage yaw  -15 degree direction \n"<<endl;
                break;
            case 107:
                //CMD=j
                cout<< "chage yaw  +15 degree direction \n"<<endl;
                break;

        }

    } while (CMD != 'q');
    cout << "quit" << endl;
//방법 1
//join()을 실행시키면 t가 종료되기 전까지 기다린다.
    return 0;
}