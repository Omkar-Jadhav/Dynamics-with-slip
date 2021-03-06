function [vdot_states Vdot omega_dot t1_ddot t2_ddot sr sl Tr Tl flong_1 flong_2] = ...
    EOM_with_slip(t,v,R,Meq,mr,mewR,mewL,b,i,Jeq,Tr,Tl,P1,P2,L,Iwy,N,F_roll)
%EOM_WITH_SLIP Summary of this function goes here
%   Detailed explanation goes here
V=v(1);
omega=v(2);
t1_dot=v(3);
t2_dot=v(4);

rho1_dot=(V+omega*L)/2;
rho2_dot=(V-omega*L)/2;


%% Impulse input
if( t>0 && t<0.001)
    Tr=1000;
    Tl=1000;
end

% if(V<0)
%     F_roll=-1*F_roll;
% end

%% Longitudnal forces
%Right wheel
if( rho1_dot> R*t1_dot) %Case of decelerating
        sr=(( R*t1_dot- rho1_dot)/(rho1_dot));
        flong_1=friction(mewR,sr,N);
        
elseif( rho1_dot< (R*t1_dot)) % Case of accelerating
    
        sr=(( R*t1_dot- rho1_dot)/(R*t1_dot));
        flong_1=friction(mewR,sr,N);
else
     flong_1=0;
     sr=0;
end

%Left wheel
if( rho2_dot> R*t2_dot) %Case of decelerating
        sl=((R*t2_dot)- rho2_dot)/(rho2_dot);
        flong_2=friction(mewL,sl,N);
        
elseif( rho2_dot)< (R*t2_dot)) % Case of accelerating
        sl=(( R*t2_dot- rho2_dot)/(R*t2_dot));
        flong_2=friction(mewL,sl,N);
else
    
     flong_2=0;
     sl=0;
end

s=[sr;sl];
%% wheel EOMs 
% t1_ddot=(Tr-flong_1*R)/Iwy;
% t2_ddot=(Tl-flong_2*R)/Iwy;


t1_ddot=(Tr-flong_1*R-F_roll*R)/Iwy;
t2_ddot=(Tl-flong_2*R-F_roll*R)/Iwy;
%% Body EOM
% Vdot= [(Tr+Tl)/R+mr*b*omega^2-(Iwy/R)*(t1_ddot+t2_ddot) ]/(2*(P1+P2));
% omega_dot=[( (Tr-Tl)/R- mr*b/L*V*omega -(Iwy/R)*(t1_ddot-t2_ddot) )]/(2*L*(P1-P2));

Vdot= [mr*b*omega^2+(flong_1+flong_2-2*F_roll)]/(2*(P1+P2));
omega_dot=(- mr*b/L*V*omega+(flong_1-flong_2) )/(2*L*(P1-P2));

% 
% Vdot= [mr*b*omega^2+(flong_1+flong_2)]/(2*(P1+P2));
% omega_dot=(- mr*b/L*V*omega+(flong_1-flong_2) )/(2*L*(P1-P2));
%% Output states
vdot_states=[Vdot;omega_dot;t1_ddot;t2_ddot];
end

