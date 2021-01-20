%% 
clc
clear all
close all
%% Defining variables
syms rho1 rho2 t1 t2 eta1 tau1 tau2 omega V T_d T_t mr b vr omega_r R ...
        t1_ddot t2_ddot t1_dot t2_dot L Vdot omega_dot Tr Tl Meq Jeq
mc = 27;                      % Mass of the Robot platform only
mr=mc;
mw=0.5;                      % Mass of the wheel and motor
R = 0.0975;                  % Radius of the wheel
b= 0.05;                     % Distance of the robot from the center of drive axle
L = 0.164;                   % Distance between two wheels

Im=0.025/2;                  %MI of the wheel about diametrical axis
Iwy=0.025;                   %MI of the wheel about the wheel axis
Ic=0.732;                    %MI of only the robot platform w.r.t. CG
I=Ic+mc*b^2+2*mw*L^2+2*Im;   %MI of robot wrt centre point A.
m=mc+2*mw;                   %Total mass of the robot


P1=(m*L^2+I)/(4*L^2);        %Term1 from dynamic equations with contact patch
P2=(m*L^2-I)/(4*L^2);        %Term2 from dynamic equations with contact patch

Meq=m+(2*Iwy/R^2);           %Equivalent mass term in dynamics with point contact
Jeq=(I+2*Iwy*L^2/R^2);       %Equivalent inertial term in dynamics with point contact

N=m*9.81/2;     %Normal force on each wheel
mewR=0.8;       %Initial mew of right wheel
mewL=0.8;       %Initial mew of left wheel
mew_change=0.5; %What is the change in mew? 
dist='N'        %Add the disturbance or not?  -----> 'Y' for yes

Cr=0.01;
F_roll=Cr*N;
%% Ode45
timespan=0:0.01:150;
Vr=1;                   %Reference linear velocity
omega_r=0;              %Reference angular velocity
 
wn=0.;                 %Natual frequency of the system
zeta=2;                 %Damping ratio of the system

Kiv=wn^2;       %Integral gain for linear velocity
Kiw=wn^2;       %Integral gain for angular velocity

Kv=2*wn*zeta;   %Proportional gain for linear velocity
Kw=2*wn*zeta;   %Proportional gain for angular velocity

% Solver
Tr=0;
Tl=0;
%Integrating states [Vdot omega_dot t1_ddot t2_ddot]
[t,v]=ode23s(@(t,v) EOM_with_slip_s_abs(t,v,R,Meq,mr,mewR,mewL,b,i,Jeq,...
                            Tr,Tl,P1,P2,L,Iwy,N,F_roll),timespan,[0;0;0;0]);
%% Retracing the intermediate values for plotting
[~,Vdot omega_dot t1_ddot t2_ddot sr sl Tr Tl flong_1 flong_2] = cellfun(@(t,v)  EOM_with_slip_s_abs(t,v,R,Meq,mr,mewR,mewL,b,i,Jeq,...
                            Tr,Tl,P1,P2,L,Iwy,N,F_roll), num2cell(t),...
                                        num2cell(v,2),'uni',0);
Vdot=cell2mat(Vdot);
omega_dot=cell2mat(omega_dot);
V=v(:,1);
omega=v(:,2);
rho1_dot=(V+omega*L)/2;
rho2_dot=(V-omega*L)/2;
t1_dot=v(:,3);
t2_dot=v(:,4);
t1_ddot=cell2mat(t1_ddot);
t2_ddot=cell2mat(t2_ddot);
sr=cell2mat(sr);
sl=cell2mat(sl);
Tr=cell2mat(Tr);
Tl=cell2mat(Tl);
flong_1=cell2mat(flong_1);
flong_2=cell2mat(flong_2);
T=[Tr Tl];
Vdot_states=[Vdot omega_dot t1_ddot t2_ddot];
s=[sr sl];

%% plots
Case=1;
if(Vr==1 && omega_r==0)
    Name='For step $V$'
    fpath='D:\From open loop\Simulation\Dynamic with slip\Controller response with slip\Step V';
end
if(Vr==0 && omega_r==1)
    Name='For step $\omega$'
    fpath='D:\From open loop\Simulation\Dynamic with slip\Controller response with slip\Step w';

end
if(Vr==1 && omega_r==1)
    Name='For step $V$ \& step $\omega$'
    fpath='D:\From open loop\Simulation\Dynamic with slip\Controller response with slip\Step V & w';

end
if(dist=='Y')
    Name='For ramp $V$ with disturbance'
    fpath='D:\From open loop\Simulation\Dynamic with slip\Controller response with slip\With disturbances\Ramp V';
end
Save='N';   
Plots(t,v,Vdot_states,s,T,fpath,Iwy,R,Vr,omega_r,Kv,Kw,mr,b,P1,P2,L,Save,Name,dist);