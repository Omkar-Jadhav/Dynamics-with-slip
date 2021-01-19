clc
clear all
close all

s=[-1:0.01:1];
% For dry turmac
mew1=0.8;
for i=1:length(s)
    if(s(i)<=0.151 && s(i)>=-0.15)
       f(i)=s(i)*mew1/0.15;
    elseif(s(i)>= 0.15 && s(i)<=1 )
      f(i)=-(s(i)-0.15)*0.34*mew1/0.85+mew1;
    elseif(s(i)<-0.15&& s(i)>=-1)
        f(i)= -(s(i)+0.15)*0.34*mew1/0.85-mew1;
    end
end

%For snow
mew2=0.3;
for i=1:length(s)
    if(s(i)<=0.151 && s(i)>-0.15)
       f2(i)=s(i)*mew2/0.15;
    elseif(s(i)> 0.151 && s(i)<=1 )
      f2(i)=-(s(i)-0.15)*0.34*mew2/0.85+mew2;
    elseif(s(i)<-0.15&& s(i)>=-1)
        f2(i)= -(s(i)+0.15)*0.34*mew2/0.85-mew2;
    end
end

plot(s,f,'Linewidth',2)
hold on
plot(s,f2,'Linewidth',2)
% xline(0.1)
legend({'$\mu_{max}=0.8$','$\mu_{max}=0.3$'},'Location','Northwest','Interpreter','Latex','FontSize',14)
grid on
box on 
xlabel({"Slip ratio"},'Interpreter','latex','FontSize',14)
ylabel({"Friction"},'Interpreter','latex','FontSize',14)
title({"Friction vs slip ratio"},'Interpreter','latex','FontSize',14)