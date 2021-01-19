function [vdot_states s] = EOM_with_slip_s_abs(t,v,R,Meq,mr,mewR,mewL,b,i,Jeq,Tr,Tl,P1,P2,L,Iwy,N)
%EOM_WITH_SLIP Summary of this function goes here
%   Detailed explanation goes here
V=v(1);
omega=v(2);
t1_dot=v(3);
t2_dot=v(4);

% V_dot=v(5);
% w_dot=v(6);
% t1_ddot=v(7);
% t2_ddot=v(8)
rho1_dot=(V+omega*L)/2;
rho2_dot=(V-omega*L)/2;

%% Longitudnal forces
%Right wheel
if(abs(rho1_dot)>abs(R*t1_dot)) %Case of braking
    if(abs(rho1_dot)<10e-7)
        sr=0;
        flong_1=0;
    else
        sr=((abs(R*t1_dot)-abs(rho1_dot))/(rho1_dot));
        flong_1=friction(mewR,sr,N);
    end
    
elseif(abs(rho1_dot)<abs((R*t1_dot))) % Case of accelerating
    if(abs(R*t1_dot)<10e-7)
        flong_1=0;
        sr=0;
    else
        sr=((abs(R*t1_dot)-abs(rho1_dot))/(R*t1_dot));
        flong_1=friction(mewR,sr,N);
    end
else
     flong_1=0;
     sr=0;
end

%Left wheel
if(abs(rho2_dot)>abs(R*t2_dot)) %Case of braking
    if(abs(rho2_dot)<10e-7)
        flong_2=0;
        sl=0;
    else
        sl=((abs(R*t2_dot)-abs(rho2_dot))/(rho2_dot));
        flong_2=friction(mewL,sl,N);
    end
elseif(abs(rho2_dot)<abs((R*t2_dot))) % Case of accelerating
    if(abs(R*t2_dot)<10e-7)
        flong_2=0;
        sl=0;
    else
        sl=((abs(R*t2_dot)-abs(rho2_dot))/(R*t2_dot));
        flong_2=friction(mewL,sl,N);
    end
else
     flong_2=0;
     sl=0;
end

s=[sr;sl];
%% wheel EOMs 
t1_ddot=(Tr-flong_1*R)/Iwy;
t2_ddot=(Tl-flong_2*R)/Iwy;
%% Body EOM
Vdot= [(Tr+Tl)/R+mr*b*omega^2-(Iwy/R)*(t1_ddot+t2_ddot) ]/(2*(P1+P2));
omega_dot=( (Tr-Tl)/R- mr*b/L*V*omega -(Iwy/R)*(t1_ddot-t2_ddot) )/(2*L*(P1-P2));
%% Output states

vdot_states=[Vdot;omega_dot;t1_ddot;t2_ddot];
end

