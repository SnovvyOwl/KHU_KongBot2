#include<Matrix.h>
#include<iostream>
#include<vector>
#include<fstream>
using namespace std;
double KF_filter(Matrix<double>& m,double state, double ex_state);
int main(){
    ofstream fout;
    fout.open("data.txt");
    vector<vector<double>> tmp  
    {
        {0,1,0,0,0,0},
        {24.65561834, 0.27506834, 24.65561834, 0, 24.65561834, 0},        
        {0, 0, 0, 1, 0, 0},        
        {-24.65561834, -0.27506834, -24.65561834, 0, -24.65561834,0},       
        {0, 0, 0, 0, 0, 1},
        {-123.42648199, -0.37699722, -123.42648199, 0, -123.42648199, 0}
    };
    Matrix<double>A(6,6,tmp);
    
    return 0;
}
double KF_filter(Matrix<double> &m,double state, double ex_state){
    double filtered_state=0;
    return filtered_state;
}