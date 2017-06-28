% cruisedyn - dynamic model for cruise control example
%
% Usage: acc = cruisedyn(v, u, gear, theta, m)
% 
% This function returns the acceleration of the vehicle given the 
% current velocity (in m/s), throttle setting (0 <= u <= 1), gear (1-5),
% road slope (default 0) and car mass (default 1000 kg).

% RMM, 2 Jul 06

function dv = cruisedyn(v, u, gear, theta, m)

% Check the parameters to make sure that they are OK
% MISSING

% Parameters for defining the system dynamics
alpha = [40, 25, 16, 12, 10];		% gear ratios
Tm = 190;				% engine torque constant, Nm
wm = 420;				% peak torque rate, rad/sec
beta = 0.4;				% torque coefficient
Cr = 0.01;				% coefficient of rolling friction
rho = 1.3;				% density of air, kg/m^3
Cd = 0.32;				% drag coefficient
A = 2.4;				% car area, m^2
g = 9.8;				% gravitational constant

% Set defaults if all arguments aren't passed
if (nargin < 3) gear = 1; end;	% gear ratio
if (nargin < 4) theta = 0; end;	% road angle
if (nargin < 5) m = 1000; end;	% mass of the car

% Check to make sure arguments are reasonable
if (gear < 1 || gear > 5)
  fprintf(2, 'cruisedyn: gear ratio out of range');
  acc = 0; return;
end

if (theta < -pi/4 || theta > pi/4)
  fprintf(2, 'cruisedyn: road slope out of range\n');
  acc = 0; return;
end

if (m < 500 || m > 10000) 
  fprintf(2, 'cruisedyn: vehicle mass out of range\n');
  acc = 0; return;
end

% Saturate the input to the range of 0 to 1
u = min(u, 1);  u = max(0, u);

% Compute the torque produced by the engine, Tm
omega = alpha(gear) * v;		% engine speed
torque = u * Tm * ( 1 - beta * (omega/wm - 1)^2 );
F = alpha(gear) * torque;

% Check to make sure engine settings are reasonable
if (omega > 700)
  fprintf(2, 'cruisedyn: engine rpm too high (%g)\n', omega*30/pi);
end

if (torque < 0) 
  fprintf(2, 'cruisedyn: torque is negative (u=%g, omega=%g)\n', u, omega);
end

% Compute the external forces on the vehicle
Fr = m * g * Cr;			% Rolling friction
Fa = 0.5 * rho * Cd * A * v^2;		% Aerodynamic drag
Fg = m * g * sin(theta);		% Road slope force
Fd = Fr + Fa + Fg;			% total deceleration

% Now compute the acceleration 
dv = (F - Fd) / m;

