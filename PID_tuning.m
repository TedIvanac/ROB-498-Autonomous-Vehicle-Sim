clc;
clear;
close all;
clear PID_sys
clear PID_tuning

%Starting state variables
X = 0;
Y = 0;
Theta = 0;
Delta = 0;
v = 4; %cannot start at v=0 ?
e_v = 0;
e_s = 0;

% Time parameters
t_end = 60;
% WaypointlistX = [30, 41, 35, 25,60, 80, 130]; %(m)
% WaypointlistY = [0, 10, 20, 45, 70,15, -20]; %(m)

WaypointlistX = [30, 41, 35, 25, 60, 80, 130]; %(m)
WaypointlistY = [0, 10, 20, 45, 70, 15, -20]; %(m)

%Creating an event
[t, output_state] = ode45(@PID_sys, [0,t_end], [X;Y;Theta;v; e_v; e_s]);
% Initialize arrays to store results
X_history = output_state(:,1);
Y_history = output_state(:,2);
Theta_history = output_state(:,3);
velocity_history = output_state(:,4);

% Plot results
figure;
subplot(3, 1, 1);
plot(t, X_history);
xlabel('Time (s)');
ylabel('X Position (m)');
title('Bicycle Model Simulation');

subplot(3, 1, 2);
plot(t, Y_history);
xlabel('Time (s)');
ylabel('Y Position (m)');

% subplot(4, 1, 3);
% plot(t, Theta_history);
% xlabel('Time (s)');
% ylabel('Yaw Angle (rad)');

subplot(3, 1, 3);
plot(t, velocity_history);
xlabel('Time (s)');
ylabel('Velocity (m/s)');

figure
hold on
plot(X_history,Y_history)
plot(WaypointlistX, WaypointlistY, 'o');
title('Bicycle Model Trajectory with Steering Controller');
xlabel('X Position (m)');
ylabel('Y Position (m)');