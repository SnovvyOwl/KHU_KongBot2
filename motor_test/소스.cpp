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
		time = data.substr(7, data.find("E") - 7);
		pos = data.substr((data.find("s") + 4), (data.find("A") - 1) - (data.find("s") + 4));
		angle = data.substr(data.find("le") + 5, (data.find("V") - 1) - (data.find("le") + 6));
		vel = data.substr(data.find("Vel") + 5);
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