% cruisepi_lin_nl.m - PI control with antiwindup for linear and nonlinear
% models
% kja 060726

aminit;

cruise_conpar;
cruise_lin;
theta_d = 4;				% set the hill angle

%
% PI controller simulation of linear and nonlinear models
%

[t, x] = ode45('cruise_clsysode', [0:0.1:30], [v_e u_e v_e u_e]);
vd = vref * ones(length(t));
vert = [18; 22];                    % vertical line

figure(1); clf; subplot(321); hold on;
h2 = plot(t, x(:,3), 'r--', 'Linewidth', AM_thick_linewidth)
h1 = plot(t, x(:,1), 'b-', 'Linewidth', AM_data_linewidth)

% Add reference lines
hold on
pl = plot(t, vd,'k-', [5, 5], vert, 'k--');
set(pl,'Linewidth', AM_ref_linewidth);

amaxis([0 30 18.8 20.5]);
ylabel('Velocity {\itv} [m/s]');

lgh = legend(gca, [h1, h2], {'Nonlinear', 'Linear'}, 'Location', 'southeast');
legend(lgh, 'boxoff'); box on;

% Compute control signal for plotting
e = vref -x(:,1);                   % speed error
u = kp * e + x(:,2);                % throttle
us=max(u,0);us=min(us,1);           % saturate throttle  
ul=kp*(vref-x(:,3))+x(:,4);         % throttle for linear model                       
usl=max(ul,0);usl=min(usl,1);

subplot(323); hold on;
plot(t, usl, 'r--', 'Linewidth', AM_thick_linewidth)
plot(t, u, 'b-', 'Linewidth', AM_data_linewidth)

hold on; pl = plot([5, 5], [0, 1], 'k--');
set(pl,'Linewidth', AM_ref_linewidth);
grid off; box on;

amaxis([0 30 0 1.0]);
xlabel('Time {\itt} [s]');
ylabel('Throttle {\itu}');

amprint('cruisepi-lin-nl-ch5.eps');

