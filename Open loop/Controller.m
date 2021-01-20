function [vdot_states Vdot omega_dot t1_ddot t2_ddot sr sl Tr Tl flong_1 flong_2]= Controller(t,v,Vr,omega_r,Kv,Kw,Kiv,Kiw,R,mr,b,Meq,Jeq,...
    mewR,mewL,mew_change,dist,P1,P2,Iwy,L,N,T_roll)
%OLD_CONTROLLER Summary of this function goes here
%   Detailed explanation goes here
V=v(1);
omega=v(2);
% Vr=Vr*t;      %Used for acceleration

Ve=Vr-V;            %Velocity errors
we=omega_r-omega;   %Velocity errors    



if(dist=='Y')       %Simulating change in surface
    if(t>40 && t<50)
        mewR=mewR-mew_change;   
    end
end

[vdot_states Vdot omega_dot t1_ddot t2_ddot sr sl Tr Tl flong_1 flong_2]=...
    EOM_with_slip_s_abs(t,v,R,Meq,mr,mewR,mewL,b,i,Jeq,Tr,Tl,P1,P2,L,Iwy,N,T_roll);
% vdot_states=EOM_without_slip(t,v,R,Meq,mr,b,Jeq,Tr,Tl,T_roll,L);
tg=t;           %updating global time variable
end

