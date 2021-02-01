function [vdot_states Vdot omega_dot t1_ddot t2_ddot sr sl Tr Tl flong_1 flong_2]= Controller(t,v,Vr,omega_r,Kv,Kw,Kiv,Kiw,R,mr,b,Meq,Jeq,...
    mewR,mewL,mew_change,dist,P1,P2,Iwy,L,N,T_roll)
%OLD_CONTROLLER Summary of this function goes here
%   Detailed explanation goes here
V=v(1);
omega=v(2);
% Vr=Vr*t;      %Used for acceleration

Ve=Vr-V;            %Velocity errors
we=omega_r-omega;   %Velocity errors    

global tg int_v_error int_w_error;   %Initializing global variables for integral terms
if(t==0)
    tg=0;
    int_v_error=0;
    int_w_error=0;
end

int_v_error=int_v_error+(t-tg)*Ve;  %Integral terms
int_w_error=int_w_error+(t-tg)*we;  %Integral terms


Td=R*[(Kv*Ve+Kiv*int_v_error)*(2*(P1+P2))-mr*b*omega^2];        %Driving torque command
Tt=R*[(Kw*we+Kiw*int_w_error)*(2*L*(P1-P2))+mr*b/L*V*omega];    %Turning torque command

% Td=R*((Kv*Ve+Kiv*int_v_error)*Meq-mr*b*omega^2) ;             %Driving torque for point contact model
% Tt=R*((Kw*we+Kiw*int_w_error)*Jeq+mr*b/L*V*omega);              %Turning torque command for point contact model


Tr=(Td+Tt)/2;       %Wheel torques
Tl=(Td-Tt)/2;       %Wheel torques


if(dist=='Y')       %Simulating change in surface
    if(t>40 && t<50)
        mewR=mewR-mew_change;   
    end
end

[vdot_states Vdot omega_dot t1_ddot t2_ddot sr sl Tr Tl flong_1 flong_2]=...
    EOM_with_slip_s_abs(t,v,R,Meq,mr,mewR,mewL,b,i,Jeq,Tr,Tl,P1,P2,L,Iwy,N,T_roll);
tg=t;           %updating global time variable
end

