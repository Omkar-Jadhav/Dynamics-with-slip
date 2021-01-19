function vdot_states = old_controller_modified(t,v,Vr,omega_r,Kv,Kw,R,mr,b,Meq,Jeq,mew,P1,P2,Iwy,i,L,N)
%OLD_CONTROLLER_MODIFIED Summary of this function goes here
%   Detailed explanation goes here
V=v(1);
omega=v(2);

Ve=Vr-V;
we=omega_r-omega;

Td=R*[Kv*Ve*(2*(P1+P2))-mr*b*omega^2];
Tt=R*[Kw*we*(2*L*(P1-P2))+mr*b/L*V*omega];


Tr=(Td+Tt)/2;
Tl=(Td-Tt)/2;


vdot_states=EOM_with_slip(t,v,R,Meq,mr,mew,b,i,Jeq,Tr,Tl,P1,P2,L,Iwy,N);
% vdot_states=EOM_without_slip(t,v,R,Meq,mr,b,Jeq,Tr,Tl);
end

