%% 
clc
clear all
close all
%% Defining variables
syms rho1 rho2 t1 t2 eta1 tau1 tau2 omega V T_d T_t mr b vr omega_r R ...
        t1_ddot t2_ddot t1_dot t2_dot L Vdot omega_dot Tr Tl Meq Jeq
mc = 2; % Mass of the Robot platform
mr=mc;
mw=0.5; % Mass of the wheel and motor
R = 0.0364; % Radius of the wheel
b= 0.05; % Distance of the robot from the center of drive axle
L = 0.164; % Distance between two wheels

% Iw= mw*R^2/2;   % MI of wheel and motor wrt wheel axis.
% Ic= mc*0.150^2        %MI of robot about COM(approx)
Im=0.0012;
Iwy=0.025;
Ic=0.732;
I=Ic+mc*b^2+2*mw*L^2+2*Im;   %MI of robot wrt centre point A.


m=mc+2*mw;
Meq=m+(2*Iwy/R^2);
Jeq=(I+2*Iwy*L^2/R^2);  
P1=(Ic + 2*Iwy + L^2*mr + 4*b^2*mw + b^2*mr);
P2=(- mr*b^2 + mr*L^2 + Ic + 2*Iwy);
K=0.8;

%% Ode45
Tr=1;
Tl=-1;
timespan=0:0.5:60;
i=0
% [t,v]=ode45(@(t,v) EOM_with_slip(t,v,R,Meq,mr,K,b,i,Jeq,Tr,Tl,P1,P2,L,Iwy),timespan,[0;0;0;0]);
[t,v]=ode45(@(t,v) EOM_without_slip(t,v,R,Meq,mr,b,Jeq,Tr,Tl),timespan,[0;0]);

%% plots
V=v(:,1);
omega=v(:,2);

subplot(1,2,1)
plot(t,V)
xlabel("Time",'Interpreter','latex');
ylabel("V",'Interpreter','latex');
title('V vs Time','Interpreter','latex')
grid on
box on

subplot(1,2,2)
plot(t,omega)
xlabel("Time",'Interpreter','latex');
ylabel("$\omega$",'Interpreter','latex');
title('$\omega$ vs Time','Interpreter','latex')
grid on
box on

fpath='D:\From open loop\Simulation\Dynamic with slip\Open loop step responses';
filename='Opposite step torque inputs';
saveas(gca, fullfile(fpath, filename), 'jpg');