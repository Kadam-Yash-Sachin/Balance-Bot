clear all;
clc;
close all;
I_pend= 0.011008;
m_pend= 0.6752;
m_cart= 0.430;
l= 0.075693;
f= 0.01;
g=9.81;

D= I_pend*(m_cart + m_pend)+m_cart*m_pend*l^2;
i= -(I_pend+m_pend*l^2)*f/D;
j= m_pend^2*g*l^2/D;
k= -m_pend*l*f/D
m= m_pend*g*l*(m_cart+m_pend)/D;

a=-i/f;
b=-k/f;

A=[0 1 0 0;0 i j 0;0 0 0 1;0 k m 0];
B=[0;a;0;b];
C=eye(4);
D=0;

sysc = ss(A, B, C, D);

sysd=c2d(sysc,0.005);

Q=[5 0 0 0; 0 1 0 0 ; 0 0 100 0; 0 0 0 10];
R=1;

[g1, x, l1] = dlqr (sysd, Q, R);

display(l1);
zplane(0,l1);