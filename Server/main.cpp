#include<server.h>
using namespace std;
int main(){
    Server ser("127.0.0.1",13000);
    ser.startServer();
    return 0;
}