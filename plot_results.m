%plot Data
subplot(231);
plot(tout,ref_speed,'b');
title('Reference Speed');
%Velocity
subplot(232);
plot(tout,velocity,'b');
title('Velocity of Vehicle');
%Throttle
subplot(233);
plot(tout,throttle_position,'b');
title('Throttle Position');
%Torque
subplot(234);
plot(tout,torque,'b');
title('Engine Torque');
% Engine Force
subplot(235);
plot(tout,engine_force,'b');
title('Engine Force');
%Net Force
subplot(236);
plot(tout,net_force,'b');
title('Net Force on Vehicle');
