function Plots(t,v,Vdot_states,s,T,fpath,Iwy,R,Vr,omega_r,Kv,Kw,mr,b,P1,P2,L,Save,Name,dist,flong_1,flong_2)
%PLOTS Summary of this function goes here
%   Detailed explanation goes here
vdot=Vdot_states(:,1);
omega_dot=Vdot_states(:,2);
t1_ddot=Vdot_states(:,3);
t2_ddot=Vdot_states(:,4);
sr=s(:,1);
sl=s(:,2);
Tr=T(:,1);
Tl=T(:,2);

V=v(:,1);
omega=v(:,2);
t1_dot=v(:,3);
t2_dot=v(:,4);

addn_term_1=(Iwy/(2*R*(P1+P2)))*(t1_ddot+t2_ddot);
addn_term_2=(Iwy/(2*L*R*(P1-P2)))*(t1_ddot-t2_ddot);

%% Vdot plots
f1=figure;
subplot(1,2,1)
plot(t,vdot,'Linewidth',2)
xlabel({"Time"},'interpreter','latex','FontSize',14);
ylabel({"$\dot{V}$"},'interpreter','latex','FontSize',14);
% xlim([0 50])
grid on
box on

title([{"$\dot{V}$ vs Time",Name}],'interpreter','latex','FontSize',14)
subplot(1,2,2)
plot(t,omega_dot,'Linewidth',2)
xlabel({"Time"},'interpreter','latex','FontSize',14);
ylabel({"$\dot{\omega}$"},'interpreter','latex','FontSize',14);
grid on
box on
% xlim([0 50])
if(Vr==1 && omega_r==0 && dist~='Y')
    ylim([-0.1 0.1])
end
title([{"$\dot{\omega}$ vs Time",Name}],'interpreter','latex','FontSize',14)
% xline(40,'--')
% xline(50,'--')
set(gcf, 'Units', 'centimeters', 'OuterPosition', [05, 5, 30, 15]);

if(Save=='Y')
    filename='V_dots.jpg';
    exportgraphics(f1, fullfile(fpath, filename),'Resolution',600);
end
%% Theta_ddot plots
f2=figure;
subplot(1,2,1)
plot(t,t1_ddot,'Linewidth',2)
xlabel("Time",'interpreter','latex');
ylabel("$\ddot{\theta_R}$",'interpreter','latex');
grid on
box on
% xlim([0 50])
% xline(40,'--')
% xline(50,'--')
title(["$\ddot{\theta_R}$ vs Time",Name],'interpreter','latex')
subplot(1,2,2)
plot(t,t2_ddot,'Linewidth',2)
xlabel("Time",'interpreter','latex');
% xline(40,'--')
% xline(50,'--')
ylabel("$\ddot{\theta_L}$",'interpreter','latex');
grid on
box on
% xlim([0 50])
title(["$\ddot{\theta_L}$ vs Time",Name],'interpreter','latex')
set(gcf, 'Units', 'centimeters', 'OuterPosition', [05, 5, 30, 15]);

if(Save=='Y')
    filename='Theta_ddots.jpg';
    exportgraphics(f2, fullfile(fpath, filename),'Resolution',300);
end
%% Velocity plots
f3=figure;
subplot(1,2,1)
hold on
plot(t,V,'Linewidth',2)
if(dist=='Y')
%     plot(t,t,'--','Linewidth',2)
%     legend({'Actual velocity','Reference velocity'},'Location','Northwest','FontSize',14)
end
% hold off
% if(Vr==0 && omega_r==1)
%     ylim([-0.01 0.01])
% end
xlabel({"Time"},'interpreter','latex','FontSize',14);
ylabel({"V"},'interpreter','latex','FontSize',14);
grid on
box on
title([{"Velocity vs Time",Name}],'interpreter','latex','FontSize',14)
subplot(1,2,2)
plot(t,omega,'Linewidth',2)
if(Vr==1 && omega_r==0 && dist~='Y')
%     ylim([-0.1 0.1])
end
xlabel({"Time"},'interpreter','latex','FontSize',14);
ylabel({"$\omega$"},'interpreter','latex','FontSize',14);
grid on
% xlim([0 50])
box on
% xline(40,'--')
% xline(50,'--')
title([{"$\omega$ vs Time",Name}],'interpreter','latex','FontSize',14)
set(gcf, 'Units', 'centimeters', 'OuterPosition', [05, 5, 30, 15]);

if(Save=='Y')
    filename='Velocities.jpg';
    exportgraphics(f3, fullfile(fpath, filename),'Resolution',600);
end
%% Additional terms plot
figure;
subplot(1,2,1)
plot(t,addn_term_1,'Linewidth',2)
xlabel("Time",'interpreter','latex')
ylabel("Additional term in $\dot{V}$",'interpreter','latex')
% xline(40,'--')
% xline(50,'--')
grid on
box on
% xlim([0 50])
title(["Additional term in $\dot{V}$",Name],'interpreter','latex')
subplot(1,2,2)
plot(t,addn_term_2,'Linewidth',2)
xlabel("Time",'interpreter','latex')
ylabel("Additional term in $\dot{\omega}$",'interpreter','latex')
% xline(40,'--')
% xline(50,'--')
grid on 
% xlim([0 50])
if(Vr==1 && omega_r==0 && dist~='Y')
    ylim([-0.1 0.1])
end
box on
title(["Additional term in $\dot{\omega}$",Name],'interpreter','latex')
set(gcf, 'Units', 'centimeters', 'OuterPosition', [05, 5, 30, 15]);

if(Save=='Y')
    filename='Additional terms in velocities in both.jpg';
    exportgraphics(gca, fullfile(fpath, filename),'Resolution',300);
end
%% Slip ratio  plots
f4=figure;
subplot(1,2,1)
plot(t,sr,'Linewidth',2)
xlabel({"Time"},'interpreter','latex','FontSize',14)
ylabel({"$s_{R}$"},'interpreter','latex','FontSize',14)
grid on
% xline(40,'--')
% xline(50,'--')
% xlim([0 50])
box on
title([{"Slip Ratio of R.W. vs Time",Name}],'interpreter','latex','FontSize',14)
subplot(1,2,2)
plot(t,sl,'Linewidth',2)
xlabel({"Time"},'interpreter','latex','FontSize',14)
% xline(40,'--')
% xline(50,'--')
ylabel({"$s_{L}$"},'interpreter','latex','FontSize',14)
grid on
% xlim([0 50])
box on
title([{"Slip Ratio of L.W. vs Time",Name}],'interpreter','latex','FontSize',14)
set(gcf, 'Units', 'centimeters', 'OuterPosition', [05, 5, 30, 15]);

if(Save=='Y')
    filename='Slip ratio.jpg';
    exportgraphics(f4, fullfile(fpath, filename),'Resolution',600);
end
%% Torque plots

f5=figure;
subplot(1,2,1)
plot(t,Tr,'Linewidth',2)
xlabel({"Time"},'interpreter','latex','FontSize',14)
ylabel({"$\tau_{R}$"},'interpreter','latex','FontSize',14)
% xline(40,'--')
% xline(50,'--')
grid on
% xlim([0 50])
box on
title([{"$\tau_R$ vs Time",Name}],'interpreter','latex','FontSize',14)
subplot(1,2,2)
plot(t,Tl,'Linewidth',2)
xlabel({"Time"},'interpreter','latex','FontSize',14)
ylabel({"$\tau_{L}$"},'interpreter','latex','FontSize',14)
grid on
% xlim([0 50])
box on
% xline(40,'--')
% xline(50,'--')
title([{"$\tau_L$ vs Time",Name}],'interpreter','latex','FontSize',14)
set(gcf, 'Units', 'centimeters', 'OuterPosition', [05, 5, 30, 15]);

if(Save=='Y')
    filename='Torques.jpg';
    exportgraphics(f5, fullfile(fpath, filename),'Resolution',600);
end

%% Control torques
f6=figure;
Td=(Tr+Tl)/2;
Tt=(Tr-Tl)/2;
subplot(1,2,1)
plot(t,Td,'Linewidth',2)
xlabel({"Time"},'interpreter','latex','FontSize',14)
ylabel({"$\tau_{d}$"},'interpreter','latex','FontSize',14)
grid on
box on
title([{"$\tau_d$ vs Time",Name}],'interpreter','latex','FontSize',14)
subplot(1,2,2)
plot(t,Tt,'Linewidth',2)
xlabel({"Time"},'interpreter','latex','FontSize',14)
ylabel({"$\tau_{t}$"},'interpreter','latex','FontSize',14)
grid on
box on
title([{"$\tau_t$ vs Time",Name}],'interpreter','latex','FontSize',14)
set(gcf, 'Units', 'centimeters', 'OuterPosition', [05, 5, 30, 15]);
if(Save=='Y')
    filename='Control torques.tiff';
    exportgraphics(f6, fullfile(fpath, filename), 'Resolution',600);
end

%% LOngitudinal forces
f7=figure;
subplot(1,2,1)
plot(t,flong_1,'Linewidth',2)
xlabel({"Time"},'interpreter','latex','FontSize',14)
ylabel({"$f_{long_{R}}$"},'interpreter','latex','FontSize',14)
grid on
box on
title([{"$f_{long_{R}}$ vs Time",Name}],'interpreter','latex','FontSize',14)
subplot(1,2,2)
plot(t,flong_2,'Linewidth',2)
xlabel({"Time"},'interpreter','latex','FontSize',14)
ylabel({"$f_{long_{L}}$"},'interpreter','latex','FontSize',14)
grid on
box on
title([{"$f_{long_{L}}$ vs Time",Name}],'interpreter','latex','FontSize',14)
set(gcf, 'Units', 'centimeters', 'OuterPosition', [05, 5, 30, 15]);
%% Wheel angular velocities
f8=figure;
subplot(1,2,1)
plot(t,t1_dot,'Linewidth',2)
xlabel({"Time"},'interpreter','latex','FontSize',14)
ylabel({"$\dot{\theta}_R$"},'interpreter','latex','FontSize',14)
grid on
box on
title([{"$\dot{\theta}_R$ vs Time",Name}],'interpreter','latex','FontSize',14)
subplot(1,2,2)
plot(t,t2_dot,'Linewidth',2)
xlabel({"Time"},'interpreter','latex','FontSize',14)
ylabel({"$\dot{\theta}_L$"},'interpreter','latex','FontSize',14)
grid on
box on
title([{"$\dot{\theta}_L$ vs Time",Name}],'interpreter','latex','FontSize',14)
set(gcf, 'Units', 'centimeters', 'OuterPosition', [05, 5, 30, 15]);

end


