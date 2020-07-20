clc;
close all;
clear all;
%MASSES
m_arduino=0.034;
m_MD=0.026;
m_Xb=0.016;
m_bat=0.200;
m_plate=0.068;
m_uplate=0.016;
m_polyrod=0.006;
m_brassrod_l=0.0098;
m_brassrod_s=0.008;
m_EM=0.022;
m_motors=0.178;
m_BB=0.04;

m_total=m_arduino+m_MD+m_Xb+m_bat+3*m_plate+m_uplate+4*m_polyrod+4*m_brassrod_l+4*m_brassrod_s+2*m_EM+2*m_motors+m_BB;


%MOMENT OF INERTIA OF INDIVIDUAL COMPONENTS

I_arduino=8.570833*10.^-6;
I_MD=3.4665*10.^-6;
I_Xb=2.133*10.^-6;
I_bat=2.46*10.^-5;
I_plate=3.64*10.^-5;
I_uplate=3.364*10.^-6;
I_polyrod=1.6085*10.^-6;
I_brassrod_l=2.98*10.^-6;
I_brassrod_s=1.699*10.^-6;
I_EM=9.625*10.^-7;
I_motors=3.046*10.^-5;
I_BB=7.697*10.^-6;

r_arduino=30*10.^-3%50.27*10^-3;
r_MD=21*10.^-3;
r_Xb=35*10.^-3%170.04*10^-3
r_bat=80*10.^-3%51.24*10.^-3;
r_plate1=47*10.^-3;
r_plate2=13*10.^-3;
r_plate3=78*10.^-3;
r_uplate=130*10.^-3;
r_polyrod=75*10.^-3;
r_brassrod_l=83*10.^-3;
r_brassrod_s=108*10.^-3;
r_EM=70.5*10.^-3;
r_motors=68.2.^-3;
r_BB=40*10.^-3;

y_arduino=105.04*10.^-3%50.27*10.^-3;
y_MD=50.27*10.^-3;
y_Xb=110.04*10.^-3%170.04*10^-3;
y_bat=165.04*10.^-3;%105.04*10.^-3;
y_plate1=27.881*10.^-3;
y_plate2=87.655*10.^-3;
y_plate3=152.425*10.^-3;
y_uplate=207.195*10.^-3;
y_polyrod=57.77*10.^-3;
y_brassrod_l=120.04*10.^-3;
y_brassrod_s=179.81*10.^-3;
y_EM=25.5*10.^-3;
y_motors=7*10.^-3;
y_BB=35.27*10.^-3;

x_arduino=0;
x_MD=0;
x_Xb=0;
x_bat=0;%-2.^-3;
x_plate1=0;
x_plate2=0;
x_plate3=0;
x_uplate=0;
x_polyrod=0;
x_brassrod_l=0;
x_brassrod_s=0;
x_EM=0;
x_motors=0;
x_BB=0;

z_arduino=30*10^-3;%17.5*10^-3;
z_MD=0;
z_Xb=-52.5*10^-3;
z_bat=0;
z_plate1=0;
z_plate2=0;
z_plate3=0;
z_uplate=0;
z_polyrod=0;
z_brassrod_l=0;
z_brassrod_s=0;
z_EM=0;
z_motors=0;
z_BB=0;

I_total=0;
%disp(I_arduino+m_arduino*r_arduino^2 )
I_total=(I_arduino+m_arduino*r_arduino^2)+ (I_MD + m_MD*r_MD^2) + (I_Xb+m_Xb*r_Xb^2)+(I_bat+m_bat*r_bat^2)+ (I_plate+m_plate*r_plate1^2+I_plate+m_plate*r_plate2^2+I_plate+m_plate*r_plate3^2) + ( I_uplate+m_uplate*r_uplate^2 )+ 4*(I_polyrod+m_polyrod*r_polyrod^2) + 4*(I_brassrod_l+m_brassrod_l*r_brassrod_l^2) + 4*(I_brassrod_s+m_brassrod_s*r_brassrod_s^2) + (2*(I_EM+m_EM*r_EM^2)) + (2*(I_motors+m_motors*r_motors^2)) + (I_BB+m_BB*r_BB^2);
disp('Inertia=');
disp(I_total);

I_total_1=(I_arduino+m_arduino*r_arduino^2)+ (I_MD + m_MD*r_MD^2) + (I_Xb+m_Xb*r_Xb^2)+(I_bat+m_bat*r_bat^2)+ (I_plate+m_plate*r_plate1^2+I_plate+m_plate*r_plate2^2+I_plate+m_plate*r_plate3^2) + ( I_uplate+m_uplate*r_uplate^2 )+ 4*(I_polyrod+m_polyrod*r_polyrod^2) + 4*(I_brassrod_l+m_brassrod_l*r_brassrod_l^2) + 4*(I_brassrod_s+m_brassrod_s*r_brassrod_s^2) + (2*(I_EM+m_EM*r_EM^2)) + (I_BB+m_BB*r_BB^2);
disp('Inertia1=');
disp(I_total_1);
disp('M_total=');
disp(m_total);
CoGy=m_arduino*y_arduino + m_MD*y_MD +m_Xb*y_Xb + m_bat*y_bat + m_plate*y_plate1 + m_plate*y_plate2 + m_plate*y_plate3 + m_uplate*y_uplate + 4*m_polyrod*y_polyrod + 4*m_brassrod_l*y_brassrod_l + 4*m_brassrod_s*y_brassrod_s + 2*m_EM*y_EM + 2*m_motors*y_motors + m_BB*y_BB;
CoGy=CoGy/m_total;

CoGx=m_arduino*x_arduino + m_MD*x_MD +m_Xb*x_Xb + m_bat*x_bat + m_plate*x_plate1 + m_plate*x_plate2 + m_plate*x_plate3 + m_uplate*x_uplate + 4*m_polyrod*x_polyrod + 4*m_brassrod_l*x_brassrod_l + 4*m_brassrod_s*x_brassrod_s + 2*m_EM*x_EM + 2*m_motors*x_motors + m_BB*x_BB;
CoGx=CoGx/m_total;

CoGz=m_arduino*z_arduino + m_MD*z_MD +m_Xb*z_Xb + m_bat*z_bat + m_plate*z_plate1 + m_plate*z_plate2 + m_plate*z_plate3 + m_uplate*z_uplate + 4*m_polyrod*z_polyrod + 4*m_brassrod_l*z_brassrod_l + 4*m_brassrod_s*z_brassrod_s + 2*m_EM*z_EM + 2*m_motors*z_motors + m_BB*z_BB;
CoGz=CoGz/m_total;

disp('CoGx=');
disp(CoGx);

disp('CoGy=');
disp(CoGy);

disp('CoGz=');
disp(CoGz);