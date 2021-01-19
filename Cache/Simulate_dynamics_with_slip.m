%% 
clear all
close all
clc 
%% Robot parameters
mc = 2; % Mass of the Robot platform
mr=mc
mw=0.5; % Mass of the wheel and motor
r = 0.0364; % Radius of the wheel
b= 0.05; % Distance of the robot from the center of drive axle
L = 0.164; % Distance between two wheels

% Iw= mw*R^2/2;   % MI of wheel and motor wrt wheel axis.
% Ic= mc*0.150^2        %MI of robot about COM(approx)
Im=0.0012
Iwy=0.0025
Ic=0.732
I=Ic+mc*b^2+2*mw*L^2+2*Im   %MI of robot wrt centre point A.

m=mc+2*mw;
Meq=m+(2*Iwy/r^2);
Jeq=(I+2*Iwy*L^2/r^2);  

P1=Meq
P2=Jeq
A=r^2/(4*L^2)*(m*L^2+I)+Iwy;
B=r^2/(4*L^2)*(m*L^2-I);
C=r^2/(2*L)*mc*b;
%% Motor parameters
Ra=1.01;
La=0.088e-3; %0.5   
Ki=12939e-3;    
% Ki=0.057
sat=12;     %12V saturation to motor

%% Inputs
V=1;
omega=0;
K=0.7
%% Control parameters
Kw=Jeq/0.8  
Kv=Meq/0.8
%% Simulate
sim_Time = 100;
limit=0.0001
sim('Only_dynamic_with_slip.slx',sim_Time);
 Simulink.sdi.view; 