function T = Give_torques(v,Vr,omega_r,Kv,Kw,R,mr,b,P1,P2,Iwy,L)
%GIVE_TORQUES Summary of this function goes here
%   Detailed explanation goes here
V=v(:,1);
omega=v(:,2);

Ve=Vr-V;
we=omega_r-omega;

Td=R*[Kv*Ve*(2*(P1+P2))-mr*b*omega.^2];
Tt=R*[Kw*we*(2*L*(P1-P2))+mr*b/L*V.*omega];


Tr=(Td+Tt)/2;
Tl=(Td-Tt)/2;
T=[Tr Tl];
end

