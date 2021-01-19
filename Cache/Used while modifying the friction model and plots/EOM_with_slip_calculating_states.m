function [vdot_states s] = EOM_with_slip_calculating_states(t,v,R,Meq,mr,mewR,mewL,mew_change,dist,b,i,Jeq,Tr,Tl,P1,P2,L,Iwy,N)
%EOM_WITH_SLIP Summary of this function goes here
%   Detailed explanation goes here
V=v(:,1);
omega=v(:,2);
t1_dot=v(:,3);
t2_dot=v(:,4);

rho1_dot=(V+omega*L)/2;
rho2_dot=(V-omega*L)/2;

%% Longitudnal forces
%Right wheel
for i=1:length(V)
    
    if(dist=='Y')
        if(t(i)>30 && t(i)<35)
            mewR=mewR-mew_change;
        end
    end
    
    if(abs(rho1_dot(i))>abs(R*t1_dot(i))) %Case of braking
        if(abs(rho1_dot(i))<10e-7)
            flong_1(i,1)=0;
            sr(i)=0;
        else
            sr(i)=((abs(R*t1_dot(i))-abs(rho1_dot(i)))./(rho1_dot(i)));
            flong_1(i,1)=friction(mewR,sr(i),N);
        end

    elseif(abs(rho1_dot(i))<abs((R*t1_dot(i)))) % Case of accelerating
        if(abs(R*t1_dot(i))<10e-7)
            flong_1(i,1)=0;
            sr(i)=0;
        else
            sr(i)=((abs(R*t1_dot(i))-abs(rho1_dot(i)))./(R*t1_dot(i)));
            flong_1(i,1)=friction(mewR,sr(i),N);
        end
    else
         flong_1(i,1)=0;
         sr(i)=0;

    end

    %Left wheel
    if(abs(rho2_dot(i))>abs(R*t2_dot(i))) %Case of braking
        if(abs(rho2_dot(i))<10e-7)
            flong_2(i,1)=0;
            sl(i)=0;
        else
            sl(i)=((abs(R*t2_dot(i))-abs(rho2_dot(i)))./(rho2_dot(i)));
            flong_2(i,1)=friction(mewL,sl(i),N);
        end
    elseif(abs(rho2_dot(i))<abs((R*t2_dot(i)))) % Case of accelerating
        if(abs(R*t2_dot(i))<10e-7)
            flong_2(i,1)=0;
            sl(i)=0;
        else
            sl(i)=((abs(R*t2_dot(i))-abs(rho2_dot(i)))./(R*t2_dot(i)));
            flong_2(i,1)=friction(mewL,sl(i),N);
        end
    else
         flong_2(i,1)=0;
         sl(i)=0;
    end
end
%% wheel EOMs 
t1_ddot=(Tr-flong_1*R)/Iwy;
t2_ddot=(Tl-flong_2*R)/Iwy;
%% Body EOM
Vdot= [(Tr+Tl)/R+mr*b*omega.^2-(Iwy/R)*(t1_ddot+t2_ddot) ]/(2*(P1+P2));
omega_dot=( (Tr-Tl)/R- mr*b/L.*V.*omega -(Iwy/R)*(t1_ddot-t2_ddot) )/(2*L*(P1-P2));

%% Output states
sr=sr';
sl=sl';
s=[sr sl];
vdot_states=[Vdot omega_dot t1_ddot t2_ddot];
end

