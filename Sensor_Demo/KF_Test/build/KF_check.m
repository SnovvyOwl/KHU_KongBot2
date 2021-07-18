type prediction.txt
type real.txt
R=readmatrix('real.txt');
P=readmatrix('prediction.txt');
A=P(:,2);
B=R(:,2);
x=[1:1001];

plot(x,A,x,B)
