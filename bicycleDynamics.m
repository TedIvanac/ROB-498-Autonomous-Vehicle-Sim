function dydt = bicycleDynamics(in)

% Extract state variables from y
theta = in(1);
delta = in(2);
v = in(3);
throttle = in(4);

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

Da = .5*rho*Cd*A*(v + uw)^2;    %Aerodynamic drag of vehicle

Xdot = v * cos(theta);
Ydot = v * sin(theta);
Omega = v * (tan(delta)/(lf+lr));
acceleration = (throttle - W*sin(theta_g) - f*W*cos(theta_g) - Da) / m;

% Create a column vector with derivatives of x, y, theta, delta, and v
dydt = [Xdot; Ydot; Omega; acceleration];

end