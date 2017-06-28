% cruise_clsysode.m - dynamics for car with PI speed controller linear and
% nonlinear models
% kja 060726

% PI control with anti-windup for linear and nonlinear models
% systems
% Nonlinear model
% x(1)	vehicle speed
% x(2)  controller state (integrator) nonlinear model
% x(3)  vehicle speed deviation v-v_e linear model
% x(4)  controller state (integrator) linear model
%
% Note: constants in cruise_conpar must be defined before calling this function

function dx=cruise_clsysode(t, x, flags)
global kp kt ki;		% controller parameters (from cruise_conpar)
global theta_d;
cruise_lin;                     %get model parameters and linearized model

% Hill parameters: 6 deg slot at time t = 5
if (t < 5) 
  theta = 0; 
elseif (t < 6)
  % Provide a linear transition to the hill
  theta = (theta_d/180 * pi) * (t-5);
else
  theta = theta_d/180 * pi;
end

%  ----------- Nonlinear Model
%PI Controller with antiwindup
e = vref -x(1);                 %speed error
uu =kp*e+x(2);                  %nominal control signal
u=min(uu,1);u=max(u,0);         %saturated control signal
dI=ki*e+kt*(u-uu);              %update integral term with windup protection

% Nonlinear model
% Compute the torque produced by the engine
v=x(1);
omega = an * v;                 %engine angular velocity
torque = u * Tm * ( 1 - bbeta * (omega/wm - 1)^2 );
F = an * torque;
Fd=m*g*Cr+rho*Cd*A*v^2/2+m*g*sin(theta);
dv=(F-Fd)/m;                    %compute dv/dt for nonlinear model

%  ----------- Linear Model
el=vref-x(3);                   %speed error
uul=kp*el+x(4);                 %nominal control signal
ul=min(uul,1);uul=max(uul,0);   %saturated control signal
dvl=a*(x(3)-v_e)+b*(ul-u_e)-bg*theta;       %compute dv/dt for linear model
dIl=ki*el+kt*(ul-uul);          %compute dI/dt

dx = [dv;dI;dvl;dIl];

