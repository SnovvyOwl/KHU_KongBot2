#include<iostream>
using namespace std;

class Pendulum{
    private:
        int input[4]={0,0,0,0};
        float pendulumTheta[4]={0,0,0,0};
    public:
        Pendulum(float state){
            pendulumTheta[0]=state;
            pendulumTheta[1]=state;
            pendulumTheta[2]=state;
            pendulumTheta[3]=state;
        }
        void clear(){
            input[0]=0;
            input[1]=0;
            input[2]=0;
            input[3]=0;
            pendulumTheta[0]=0;
            pendulumTheta[1]=0;
            pendulumTheta[2]=0;
            pendulumTheta[3]=0;
        }

        
        int observe(float curInput){
            //Pendulum Transfer Funtion  
            /*             452.2 s + 5781                                   theta_s
                --------------------------------                =    --------------------------------
                    s^3 + 38.85 s^2 + 851.1 s + 5774                        theta_p

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
};