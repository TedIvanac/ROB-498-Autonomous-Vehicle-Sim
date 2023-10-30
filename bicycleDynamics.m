function dydt = bicycleDynamics(t, input, throttle_force,delta)

% Bicycle Model Parameters
m = 1519.988;         % Mass of the bicycle and rider (kg)
Iz = 1.2;       % Moment of inertia about the vertical axis (kg*m^2)
lf = 0.9;       % Distance from the center of mass to the front wheel (m)
lr = 0.9;       % Distance from the center of mass to the rear wheel (m)
Cf = 1600;      % Front tire cornering stiffness (N/rad)
Cr = 1800;      % Rear tire cornering stiffness (N/rad)
rho = 1.2;      %density of air
Cd = .4;        %drag coeff for typical 4 door car
A = 2.2914;     %cross sectional area of a Chevrolet Camaro
uw = 4.1667;    %wind velocity
g = 9.8;        %gravitational constant
W = m*g;        %gravitational force
f = 0.02;       %coeff of rolling resistance
theta_g = 0;    %Angle of vehicle with ground

% Steering angle
delta_time = 0:0.01:10;
delta = [zeros(1,351),ones(1,300)*delta,zeros(1,350)];
delta_q = interp1(delta_time,delta, t);

%Throttle
throttle_time = 0:0.01:10;
throttle_force_array = [zeros(1,350 + 1),throttle_force*ones(1,300),zeros(1,350)];
throttle_force_q = interp1(throttle_time,throttle_force_array, t)

% Extract state variables from y
x = input(1);
y = input(2);
theta = input(3);
v = input(4);
%delta = input(4);
%throttle_force = input(5);  % Throttle force in Newtons

Da = .5*rho*Cd*A*(v + uw)^2;    %Aerodynamic drag of vehicle

% Compute lateral forces on the front and rear tires
Fyf = Cf * delta;
Fyr = -Cr * atan((lf * delta) / (lr + lf));
% 
% % Compute lateral acceleration and angular velocity
% Ay = (Fyf + Fyr) / m;
% Omega = (lf * Fyf - lr * Fyr) / Iz;

Xdot = v * cos(theta);
Ydot = v * sin(theta);
Omega = v * (tan(delta_q)/(lf+lr));
%Delta_dot = 0;
acceleration = (throttle_force_q - W*sin(theta_g) - f*W*cos(theta_g) - Da) / m;

% Create a column vector with derivatives of x, y, theta, delta, and v
dydt = [Xdot; Ydot; Omega; acceleration];

end
