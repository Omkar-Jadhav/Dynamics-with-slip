function vdot_states = EOM_without_slip(t,v,R,Meq,mr,b,Jeq,Tr,Tl,T_roll,L)

V=v(1);
omega=v(2);

if(V>0)
   Tr=Tr-T_roll;
   Tl=Tl-T_roll;
end

Vdot=(Tr+Tl)/(R*Meq)+mr*b*omega^2/Meq;
omega_dot=(L*(Tr-Tl))/(R*Jeq)-mr*b*omega*V/Jeq;

vdot_states=[Vdot;omega_dot]
end

