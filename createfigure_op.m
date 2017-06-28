function createfigure(Y1, Y2, Y3, Y4, Y5, Y6)
%CREATEFIGURE(Y1,Y2,Y3,Y4,Y5,Y6)
%  Y1:  vector of y data
%  Y2:  vector of y data
%  Y3:  vector of y data
%  Y4:  vector of y data
%  Y5:  vector of y data
%  Y6:  vector of y data

%  Auto-generated by MATLAB on 05-May-2013 17:44:34

% Create figure
figure1 = figure;

% Create subplot
subplot1 = subplot(2,3,1,'Parent',figure1);
box(subplot1,'on');
hold(subplot1,'all');

% Create plot
plot(Y1,'Parent',subplot1);

% Create title
title('REFERENCE SPEED');

% Create subplot
subplot2 = subplot(2,3,2,'Parent',figure1);
box(subplot2,'on');
hold(subplot2,'all');

% Create plot
plot(Y2,'Parent',subplot2);

% Create title
title('VELOCITY');

% Create subplot
subplot3 = subplot(2,3,3,'Parent',figure1);
box(subplot3,'on');
hold(subplot3,'all');

% Create plot
plot(Y3,'Parent',subplot3);

% Create title
title('ENGINE TORQUE');

% Create subplot
subplot4 = subplot(2,3,4,'Parent',figure1);
box(subplot4,'on');
hold(subplot4,'all');

% Create plot
plot(Y4,'Parent',subplot4);

% Create title
title('ENGINE FORCE');

% Create subplot
subplot5 = subplot(2,3,5,'Parent',figure1);
box(subplot5,'on');
hold(subplot5,'all');

% Create plot
plot(Y5,'Parent',subplot5);

% Create title
title('TOTAL FORCE');

% Create subplot
subplot6 = subplot(2,3,6,'Parent',figure1);
box(subplot6,'on');
hold(subplot6,'all');

% Create plot
plot(Y6,'Parent',subplot6);

% Create title
title('THROTTLE');
