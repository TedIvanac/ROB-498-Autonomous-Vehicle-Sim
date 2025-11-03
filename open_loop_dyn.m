clc;
clear;
close all;

% Bicycle Model Parameters
m = 1519.988;   % Mass of the bicycle and rider (kg)
lf = 0.9;       % Distance from the center of mass to the front wheel (m)
lr = 0.9;       % Distance from the center of mass to the rear wheel (m)
rho = 1.2;      %density of air
Cd = .4;        %drag coeff for typical 4 door car
A = 2.2914;     %cross sectional area of a Chevrolet Camaro
uw = 4.1667;    %wind velocity
g = 9.8;        %gravitational constant
W = m*g;        %gravitational force
f = 0.02;       %coeff of rolling resistance
theta_g = 0;    %Angle of vehicle with ground
delta = [0.1, 0.2, 0.3, -0.1, -0.2, 0]; %Steering angle (rad)
throttle_force = 100; %Throttle force in Newtons

%State variables
X = 0;
Y = 0;
Psi = 0;
v = 10;

% Time parameters
dt = 0.01;
t_end = 2;

% Initialize arrays to store results
time = 0:dt:t_end;
X_history = zeros(size(time));
Y_history = zeros(size(time));
Psi_history = zeros(size(time));
% Simulation loop
 j = 1;
for i = 1:length(time)
    Da = .5*rho*Cd*A*(v + uw)^2;    %Aerodynamic drag of vehicle
    % Calculate acceleration using Newton's second law
    acceleration = (throttle_force - W*sin(theta_g) - f*W*cos(theta_g) - Da) / m;
    % Integrate acceleration to calculate change in velocity
    delta_v = acceleration * dt;
    % Update velocity
    v = v + delta_v;
    
    %adjust delta value over time
    if mod(i,40) == 0
        j = j+1;
    end

    % Update state variables
    X = X + v*cos(Psi)*dt;
    Y = Y + v*sin(Psi)*dt;
    delta(j)
    Psi = Psi + (v*tan(delta(j))/(lr+lf))*dt;

    % Store results
    X_history(i) = X;
    Y_history(i) = Y;
    Psi_history(i) = Psi;
end

% Plot results
figure;
subplot(3, 1, 1);
plot(time, X_history);
xlabel('Time (s)');
ylabel('X Position (m)');
title('Bicycle Model Simulation');

subplot(3, 1, 2);
plot(time, Y_history);
xlabel('Time (s)');
ylabel('Y Position (m)');

subplot(3, 1, 3);
plot(time, Psi_history);
xlabel('Time (s)');
ylabel('Yaw Angle (rad)');

figure
plot(X_history,Y_history)
title('Bicycle Model Trajectory for Varied Steering Angles');
xlabel('X Position (m)');
ylabel('Y Position (m)');