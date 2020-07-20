pkg load signal;
clc;
close all;
clear all;
km=0.5;
ke=0.3819;
R=6.2;
Mp=1.0184;
Mw=0.037;
l=0.05715;
Ip=0.0061023;
Iw=0.00003125;
r=0.0325;
g=9.81;

beta=2*Mw+2*Iw/r^2+Mp;
alpha=Ip*beta+2*Mp*l^2*(Mw+Iw/r^2);

i=2*km*ke*(Mp*l*r-Ip-Mp*l^2)/(R*r^2*alpha);

j=Mp^2*g*l^2/alpha;

m=2*km*ke*(r*beta-Mp*l)/(R*r^2*alpha);

n=Mp*g*l*beta/alpha;

i1=2*km*(-Mp*l*r + Ip + Mp*l^2)/(R*r*alpha);

i2=2*km*(-r*beta+Mp*l)/(R*r*alpha);

A=[0 1 0 0; 0 i j 0; 0 0 0 1; 0 m n 0];
B=[0; i1; 0; i2];
C=eye(4);
D=[0];

Q=[0 0 0 0; 0 0 0 0 ; 0 0 10 0; 0 0 0 0];
R=1;

[K1,s,pol]=lqr(A,B,Q,R);

sys1=ss(A-B*K1,B,C,D);
step(sys1);

sysc = ss(A, B, C, D);

display(sysc);

sysd=c2d(sysc,0.01);

display(sysd);

[z,p,k]=tf2zp(sysd);


[g1, x, l] = dlqr (sysd, Q, R);
