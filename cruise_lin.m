% cruise_lin.m - Computes linearized model for the car
% kja 060726

%Get parameters for car
cruise_carpar;
cruise_opcon;

%Compute the trottle u_e required to keep velocity ve
%at slope thetae, velocity ve, and gear
an=gears(gear);
A0=Tm*an*(1-bbeta*(an*v_e/wm-1)^2);
A1=m*g*Cr+rho*Cd*A*v_e^2/2+m*g*sin(theta_e);
u_e=A1/A0;

%Compute linearized model
w=an*v_e;
T=Tm*(1-bbeta*(w/wm-1)^2);
pT=-2*bbeta*Tm*(w/wm-1)/wm;
a=(an^2*u_e*pT-rho*Cd*A*v_e)/m;
b=an*T/m;
bg=g*cos(theta_e);
