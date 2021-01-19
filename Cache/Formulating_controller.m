clc 
clear all
%% Variables
syms tau1 tau2 tau_d tau_t v ve w we ve_dot we_dot mr m mw I Ic Iwy L b rho1 ...
    rho2 rho1_dot rho2_dot t1 t2 t1_dot t2_dot t1_ddot t2_ddot mc Im R V Vdot...
    vr vr_dot wr wr_dot P1 P2 Kv Kw

tau1=(tau_d+tau_t)/2;
tau2=(tau_d-tau_t)/2;

I=Ic+mc*b^2+2*mw*L^2+2*Im;   %MI of robot wrt centre point A.

P1=(Ic + 2*Iwy + L^2*mr + 4*b^2*mw + b^2*mr);
P2=-(- mr*b^2 + mr*L^2 + Ic + 2*Iwy);

vdot= [(tau1+tau2)/R+mr*b*w^2-(Iwy/R)*(t1_ddot+t2_ddot) ]/(2*(P1+P2));
wdot=( (tau1-tau2)/R- mr*b/L*v*w -(Iwy/R)*(t1_ddot-t2_ddot) )/(2*L*(P1-P2));

vdot=simplify(vdot)
wdot=simplify(wdot)

%% Lyapunov function
ve=vr-v;
we=wr-w;
V=0.5*(ve^2+we^2);
ve_dot=vr_dot-vdot;
we_dot=wr_dot-wdot;
Vdot=ve*ve_dot+we*we_dot

%% Trial fo controller

%The term multiplied with we
we_term =(wr_dot + ((Iwy*(t1_ddot - t2_ddot))/R - tau_t/R + (b*mr*v*w)/L)/(2*L*(P1 - P2))); 
% In order to make the entire term given above as -Kw*we we can choose tau_t as -
tau_t= (Iwy*(t1_ddot - t2_ddot))+R*(b*mr*v*w)/L+  R*(2*L*(P1 - P2))*(wr_dot+Kw*we);

% Similarly for ve term
ve_term =(vr_dot - (R*b*mr*w^2 + tau_d - Iwy*t1_ddot - Iwy*t2_ddot)/(2*R*(P1 + P2)));
% In order to make the entire term given above as -Kv*ve we can choose tau_d as -
tau_d=(Iwy*(t1_ddot + t2_ddot))-R*b*mr*w^2+ (2*R*(P1 + P2)) *(vr_dot +Kv*ve);

%% Checking Lyapunov stability
Vdot=collect(simplify(subs(Vdot)),{'Kv','Kw'})
