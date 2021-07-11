#include<iostream>
using namespace std;

class Pendulum{
    private:
        int input[4]={0,0,0,0};
        float pendulumTheta[4]={0,0,0,0};
    public:
        Pendulum(float state){
            pendulumTheta[0]=state;//Current
            pendulumTheta[1]=state;//K-1
            pendulumTheta[2]=state;//K-2
            pendulumTheta[3]=state;//K-3
        }
        void clear(){
            input[0]=0;//Current
            input[1]=0;//K-1
            input[2]=0;//K-2
            input[3]=0;//K-3
            pendulumTheta[0]=0;
            pendulumTheta[1]=0;
            pendulumTheta[2]=0;
            pendulumTheta[3]=0;
        }
        int shell2pen(float curInput){
            //Pendulum Transfer Funtion  
            /*             452.2 s + 5781                                   theta_p
                --------------------------------                =    --------------------------------
                    s^3 + 38.85 s^2 + 851.1 s + 5774                        theta_s
            
            
                0.3199 z^3 + 0.5695 z^2 + 0.1791 z - 0.07042 
                --------------------------------------------      
                 z^3 - 0.1121 z^2 + 0.1891 z - 0.08002                 

                //dt = 0.1
            */
            pendulumTheta[3]=pendulumTheta[2];
            pendulumTheta[2]=pendulumTheta[1];
            pendulumTheta[1]=pendulumTheta[0];
            input[3]=input[2];
            input[2]=input[1];
            input[1]=input[0];
            input[0]=(int)curInput;            
            pendulumTheta[0]= 0.1121*pendulumTheta[1]- 0.1891*pendulumTheta[2]+0.08002*pendulumTheta[3]+0.3199*input[0]+ 0.5695*input[1] + 0.1791 *input[2] - 0.07042*input[3]; 
            return pendulumTheta[0];
        }
        float getPenTheta(){
            return pendulumTheta[0];
        }
};

class Shell{
    private:
        float shellTheta[4]={0,0,0,0};
    public:
        Shell(){}
        void clear(){
            shellTheta[0]=0;//Current
            shellTheta[1]=0;//K-1
            shellTheta[2]=0;//K-2
            shellTheta[3]=0;//K-3
        }
        void KF()   
}

class Tilt{
    private:
    
        float RCM=0;
    public:
        //생성자
        void pentheta2Rcm(){
            //수식
            //RCM=nuber
        }
}

class Idu{
    private: 
        float idutheta=0;
        float err=0;
        float pre_err=0;
    public:
        //생성자
        int control(){
            return gain;
        }
}