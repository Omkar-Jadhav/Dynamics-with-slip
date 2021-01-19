function vdot_states = EOM_with_slip(t,v,R,Meq,mr,mewR,mewL,b,i,Jeq,Tr,Tl,P1,P2,L,Iwy,N)
%EOM_WITH_SLIP Summary of this function goes here
%   Detailed explanation goes here
V=v(1);
omega=v(2);
t1_dot=v(3);
t2_dot=v(4);

mew=mewR;
rho1_dot=(V+omega*L)/2;
rho2_dot=(V-omega*L)/2;

%% Longitudnal forces 

%% Right wheel
%Case of braking
if(rho1_dot>R*t1_dot)
    if(rho1_dot<1e-3)
        flong_1=0;
    else
        s=((R*t1_dot-rho1_dot)/rho1_dot);
        flong_1=friction(mew,s,N);
    end
    
% Case of accelerating    
elseif(rho1_dot<(R*t1_dot)) 
    if((R*t1_dot)<1e-3)
        flong_1=0;
    else
        s=((R*t1_dot-rho1_dot)/(R*t1_dot));
        flong_1=friction(mew,s,N);
    end
%Slip is zero
else  
     flong_1=0;
end

%% Left wheel
if(rho2_dot>R*t2_dot) %Case of braking
    if(rho2_dot<1e-3)
        flong_2=0;
    else
        s=((R*t2_dot-rho2_dot)/rho2_dot);
        flong_2=friction(mew,s,N);
    end
elseif(rho2_dot<(R*t2_dot)) % Case of accelerating
    if((R*t2_dot)<1e-3)
        flong_2=0;
    else
        s=((R*t2_dot-rho2_dot)/(R*t2_dot));
        flong_2=friction(mew,s,N);
    end
else
     flong_2=0;
end
%% wheel EOMs 
t1_ddot=(Tr-flong_1*R)/Iwy;
t2_ddot=(Tl-flong_2*R)/Iwy;
%% Body EOM
Vdot= [(Tr+Tl)/R-mr*b*omega^2-(Iwy/R)*(t1_ddot+t2_ddot) ]/(2*(P1+P2));
omega_dot=( (Tr-Tl)/R- mr*b/L*V*omega -(Iwy/R)*(t1_ddot-t2_ddot) )/(2*L*(P1-P2));

%% Output states
vdot_states=[Vdot;omega_dot;t1_ddot;t2_ddot];
end

