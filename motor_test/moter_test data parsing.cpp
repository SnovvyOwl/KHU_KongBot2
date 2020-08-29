#include<iostream>
#include<fstream>
#include<string>
#include<sstream>
using namespace std;
int main() {
	ifstream fin;
	fin.open("encoder_data.txt");
	string time;
	string angle;
	string vel;
	string pos;
	string data;
	ofstream tout;
	ofstream aout;
	ofstream pout;
	ofstream vout;
	tout.open("time.csv");
	aout.open("angle.csv");
	vout.open("vel.csv");
	pout.open("pos.csv");
	while (getline(fin, data)) {
		time = data.substr(data.find("@")+1, data.find("E") - data.find("@")-2);
		pos = data.substr((data.find("*") + 1), data.find("A") - data.find("*")-2);
		angle = data.substr(data.find("$") + 1, data.find("V") - data.find("$") -2);
		vel = data.substr(data.find("#") + 1);
		tout << time << endl;
		aout << angle << endl;
		vout << vel << endl;
		pout << pos << endl;
	}
	tout.close();
	fin.close();
	vout.close();
	pout.close();
	aout.close();
	return 0;
}