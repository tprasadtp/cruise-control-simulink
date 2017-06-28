% Controller parameters
% kja 060802
% Special version to simulate windup and antiwindup
% For windup set kt=0 for antiwindup set kt=2;

global kp ki kt
global theta_d

%kp = 1; ki = 0.5;			    % controller gains

kp = 0.3; ki = 0.1;	kt=0.05;	% controller gains	
a=-0.0142;b=1.3785;
%a=0.004;b=3.61;

w0=0.4;z=1;
kp=(2*z*w0+a)/b;
ki=w0^2/b;

kp=0.5;ki=0.1;kt=2;
