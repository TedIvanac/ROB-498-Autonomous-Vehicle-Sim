function dydt = full_sys(time, input)

%persistent condition
persistent WaypointlistX
persistent WaypointlistY
persistent onStart

probObstacle = rand();

if isempty(onStart)
    onStart = 1;
    condition = 0;
    generic = load('pathfile.mat', 'blankWay');
    points = waypoints(generic.blankWay, 5, 0);
    WaypointlistX = points(:,1);
    WaypointlistY = points(:,2);

elseif onStart == 1 && probObstacle > .35 && time > 1.0
    onStart = 2;
    disp(time);
    disp(probObstacle);
    generic = load('pathfile.mat', 'obstacleWay');
    points = waypoints(generic.obstacleWay, 5, 0);
    WaypointlistX = points(:,1);
    WaypointlistY = points(:,2);
end

speed = 7;

% State Variables
X = input(1);         % X position (m)
Y = input(2);          % Y position (m)
Theta = input(3);      % Yaw angle (rad)
v = input(4);
err_throttle_int = input(5);
err_theta_int = input(6);

    %Stop car if all waypoints have been hit
    if (WaypointlistX ~= inf)
        xdiff = WaypointlistX - X;
        ydiff = WaypointlistY - Y;
        ang = atan2(ydiff,xdiff);

        error = ang-Theta;
        diff = sqrt((xdiff.^2)+(ydiff.^2));
        if(diff <= .5)
            %disp("Waypoint Reached");
            points = waypoints(NaN, NaN, 1);
            WaypointlistX = points(:,1);
            WaypointlistY = points(:,2);
        end
        vel_ctrl = velocity_control(speed,v,err_throttle_int);
        steer_ctrl = steering_control(error,err_theta_int);
        throttle = vel_ctrl(1);
        err_throttle = vel_ctrl(2);
        delta = steer_ctrl(1);
            if delta > 0.7854
                delta = 0.7854;
            elseif delta < -0.7854
                delta = -0.7854;
            end
        err_theta = steer_ctrl(2);
        out = bicycleDynamics([Theta;delta;v;throttle]); 
    else
        %disp("All Waypoints Reached!")
        err_throttle = 0;

        err_theta = 0;
        out = [0;0;0;.02];
        waypoints(NaN,NaN,2);
    end
    dydt = [out; err_throttle;err_theta];
end