function [vdot_states s] = use_old_controller(t,v,Vr,omega_r,Kv,Kw,Kiv,Kiw,R,mr,b,Meq,Jeq,mewR,mewL,mew_change,dist,P1,P2,Iwy,i,L,N,m)
%OLD_CONTROLLER Summary of this function goes here
%   Detailed explanation goes here
V=v(:,1);
omega=v(:,2);

Ve=Vr-V;
we=omega_r-omega;
global tg int_v_error int_w_error errors;
if(t==0)
    tg=0;
    errors=0;
    int_v_error=0;
    int_w_error=0;
end

Ve=Vr-V;
we=omega_r-omega;
int_v_error=int_v_error+(t-tg)*Ve;
int_w_error=int_w_error+(t-tg)*we;


Td=R*[(Kv*Ve+Kiv*int_v_error)(2*(P1+P2))-mr*b*omega.^2];
Tt=R*[(Kw*we+Kiw*int_w_error)(2*L*(P1-P2))+mr*b/L*V.*omega];

Tr=(Td+Tt)/2;
Tl=(Td-Tt)/2;

tg=t;
[vdot_states s]=EOM_with_slip_calculating_states(t,v,R,Meq,mr,mewR,mewL,mew_change,dist,b,i,Jeq,Tr,Tl,P1,P2,L,Iwy,N);
% vdot_states=EOM_without_slip(t,v,R,Meq,mr,b,Jeq,Tr,Tl,T_roll,L);
end

