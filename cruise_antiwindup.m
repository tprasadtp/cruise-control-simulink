% cruisepi_lin_nl_ch9.m - PI control with antiwindup for linear and nonlinear
% models
% kja 060802
% To generate figures with different controller parameters make changes in
% cruise_conpar. With kt=0 there is no windup protection and with kt=2
% there is windup protection

aminit;

cruise_conpar;
cruise_lin;
theta_d=6;          %slope of road deg

%
% PI controller with no windup protection
%
kt = 0;
[t, x] = ode45('cruise_clsysode', [0 70], [v_e u_e v_e u_e]);
vd = vref * ones(length(t));
vert = [18; 22];                    % vertical line

figure(1); clf; subplot(321); hold on;
plot(t, vd, 'k-', [5, 5], vert, 'k:', 'Linewidth', AM_ref_linewidth);
plot(t, x(:,1), '-', 'Linewidth', AM_data_linewidth);
amaxis([0 70 18 21.5]);  box on;
% a=gca; set(a,'FontSize',16);
ylabel('velocity (m/s)');

% Compute control signal for plotting
e = vref -x(:,1);                   % speed error
u = kp * e + x(:,2);                % throttle
us=max(u,0);us=min(us,1);           % saturate throttle  
ul=kp*(vref-x(:,3))+x(:,4);         % throttle for linear model                       
usl=max(ul,0);usl=min(usl,1);

subplot(323); hold on
plot(t, u, '--', 'LineWidth', AM_data_linewidth, 'Color', [0 0.5 0]);
plot(t, us, 'b-', 'LineWidth', AM_data_linewidth);
plot([5 5], [0 2.5], 'k:', 'Linewidth', AM_ref_linewidth);

amaxis([0 70 0 2]);  box on;
xlabel('time (sec)');
ylabel('throttle');

lgh = legend('Commanded', 'Applied');
legend(lgh, 'boxoff');

amprint('cruisepi-windup.eps');

%
% PI controller with windup protection
%
kt = 2;
[t, x] = ode45('cruise_clsysode', [0 70], [v_e u_e v_e u_e]);
vd = vref * ones(length(t));
vert = [18; 22];                    % vertical line

figure(2); clf; subplot(321); hold on;
plot(t, vd, 'k-', [5, 5], vert, 'k:', 'Linewidth', AM_data_linewidth);
plot(t, x(:,1), 'b-', 'Linewidth', AM_data_linewidth);
amaxis([0 70 18 21.5]);  box on;
ylabel('velocity (m/s)');

% Compute control signal for plotting
e = vref -x(:,1);                   % speed error
u = kp * e + x(:,2);                % throttle
us=max(u,0);us=min(us,1);           % saturate throttle  
ul=kp*(vref-x(:,3))+x(:,4);         % throttle for linear model                       
usl=max(ul,0);usl=min(usl,1);

subplot(323); hold on;
plot(t, u, '--', 'LineWidth', AM_data_linewidth, 'Color', [0 0.5 0]);
plot(t, us,'-', 'Linewidth', AM_data_linewidth)
plot([5 5], [0 2.5], 'k:', 'Linewidth', AM_ref_linewidth);

amaxis([0 70 0 2]); box on;
xlabel('time (sec)');
ylabel('throttle');

lgh = legend('Commanded', 'Applied');
legend(lgh, 'boxoff');

amprint('cruisepi-antiwindup.eps');

