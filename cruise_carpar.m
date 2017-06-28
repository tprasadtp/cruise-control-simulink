% cruise_carpar.m - Parameters for defining system dynamics for the car
% kja 060727

gears = [40, 25, 16, 12, 10];	% gear ratios
m=1600;                         % mass of car kg
Tm = 190;                       % engine torque constant, Nm
wm = 420;                       % peak torque rate, rad/sec
bbeta = 0.4;                    % torque coefficient
Cr = 0.01;                      % coefficient of rolling friction
rho = 1.3;                      % density of air, kg/m^3
Cd = 0.32;                      % drag coefficient
A = 2.4;                        % car area, m^2
g = 9.8;                        % gravitational constant