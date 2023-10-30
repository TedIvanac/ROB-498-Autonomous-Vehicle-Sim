clc;
clear;
close all;

%Simulation with this model (nonlinear)

% Bicycle Model Parameters
m = 70;         % Mass of the bicycle and rider (kg)
% Iz = 1.2;       % Moment of inertia about the vertical axis (kg*m^2)
lf = 0.9;       % Distance from the center of mass to the front wheel (m)
lr = 0.9;       % Distance from the center of mass to the rear wheel (m)
% Cf = 1600;      % Front tire cornering stiffness (N/rad)
% Cr = 1800;      % Rear tire cornering stiffness (N/rad)
throttle_force = 3000;  % Throttle force in Newtons

% State Variables
X = 0;          % X position (m)
Y = 0;          % Y position (m)
Theta = 0;      % Yaw angle (rad)
Delta = pi/36;
V = 2;

% Time Parameters
dt = 0.01;      % Time step (s)
t_end = 10;     % Simulation time (s)

% Initialize arrays to store results
time = 0:dt:t_end;

%Replace sim loop
[t,output_state] = ode45(@(t,output_state) bicycleDynamics(t,output_state,throttle_force,Delta), [0, t_end], [X;Y;Theta;V]);
%[t,y] = ode45(@(t,y) odefcn(t,y,A,B), tspan, y0);

X_history = output_state(:,1);
Y_history = output_state(:,2);
Psi_history = output_state(:,3);
velocity_history = output_state(:,4);

% Plot results
figure;
subplot(4, 1, 1);
plot(t, X_history);
xlabel('Time (s)');
ylabel('X Position (m)');
title('Bicycle Model Simulation for Steering Angle = ',num2str(Delta));

subplot(4, 1, 2);
plot(t, Y_history);
xlabel('Time (s)');
ylabel('Y Position (m)');

subplot(4, 1, 3);
plot(t, Psi_history);
xlabel('Time (s)');
ylabel('Yaw Angle (rad)');

subplot(4, 1, 4);
plot(t, velocity_history);
xlabel('Time (s)');
ylabel('Velocity (m/s)');

figure
plot(X_history,Y_history)
title('Bicycle Model Trajectory');
xlabel('X Position (m)');
ylabel('Y Position (m)');