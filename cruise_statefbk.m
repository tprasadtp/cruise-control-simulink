% cruise_integralaction.m - effect of integral action
% simulation of linear model with P and PI control
% kja 070420

aminit;
cruise_conpar;           %Get controller parameters
cruise_lin               %Get parameters of linear model

%Simulation parameters
tmax=40;dt=tmax/1000;
t=0:dt:tmax;
%Generate input
u=max(0,t-5);u=min(u,1);
dd=u*4*pi/180;

%Dynamics with P control
Ap=[a-b*kp 0;-1 0];
Bp=[-bg;0];
Cp=[1 0;-kp 0];
Dp=[0;0];

%Dynamics with PI control
A=[a-b*kp b*ki;-1 0];
B=[-bg;0];
C=[1 0;-kp ki];
D=[0;0];

%Simulate systems with P and PI control
yp=lsim(Ap,Bp,Cp,Dp,dd,t);
ypi=lsim(A,B,C,D,dd,t);

%Plot
subplot(321)
pl=plot(t,yp(:,1)+20,'b--',t,ypi(:,1)+20,'b-');
set(pl,'Linewidth', AM_data_linewidth);
ylabel('v');xlabel('t');
amaxis([0 40 18 20.3]);

subplot(322)
pl=plot(t,yp(:,2)+0.2,'b--',t,ypi(:,2)+0.2,'b-');
set(pl,'Linewidth', AM_data_linewidth);
ylabel('throttle');xlabel('t');
amaxis([0 40 0 1.0]);
lgh = legend('Proportional', 'PI control', 'Location', 'southeast');
legend(lgh, 'boxoff');

amprint('cruise-integralaction.eps');
