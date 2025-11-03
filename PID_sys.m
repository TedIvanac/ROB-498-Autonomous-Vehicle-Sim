function dydt = PID_sys(time, input)
    persistent WaypointlistX
    persistent WaypointlistY
    persistent init

    % Check if the persistent variables need to be initialized
    if (isempty(init))
        init = 1;
        v = input(4);
        err_throttle = input(5);
        delta = 0;
        err_theta = input(6);
        % WaypointlistX = [30, 41, 35, 80, 130]; %(m)
        % WaypointlistY = [0, 10, 20, 15, -20]; %(m)
        WaypointlistX = [30, 41, 35, 25, 60, 80, 130]; %(m)
        WaypointlistY = [0, 10, 20, 45, 70, 15, -20]; %(m)
    end

    %State Variables
    X = input(1); %X position (m)
    Y = input(2); %Y position (m)
    Theta = input(3); %Yaw angle (rad)
    vel_setpoint = 10;

    %Stop car if all waypoints have been hit
    if(isempty(WaypointlistX))
        v = input(4);
        err_throttle = input(5);
        err_theta = input(6);
        vel_ctrl = velocity_control(0,v,err_throttle); %stop the car
        error = 0 - Theta;
        steer_ctrl = steering_control(error, err_theta); %steering wheel in neutral position
        throttle = vel_ctrl(1);
        err_throttle = vel_ctrl(2);
        delta = steer_ctrl(1);
        err_theta = steer_ctrl(2);
        out = bicycleDynamics([Theta;delta;v;throttle]);
        dydt = [out; err_throttle;err_theta];
        return
    else
        xdiff = WaypointlistX(1) - X;
        ydiff = WaypointlistY(1) - Y;
        ang = atan2(ydiff,xdiff);
        if((xdiff <= 0) && (ydiff <= 0) && (xdiff > -1) && (ydiff > -1))
            disp("Waypoint Reached");
            WaypointlistX(1) = [];
            WaypointlistY(1) = [];
            v = input(4);
            err_throttle = input(5);
            err_theta = input(6);
            error = ang-Theta;
            vel_ctrl = velocity_control(vel_setpoint,v, err_throttle);
            steer_ctrl = steering_control(error,err_theta);
            throttle = vel_ctrl(1);
            err_throttle = vel_ctrl(2);
            delta = steer_ctrl(1);
            err_theta = steer_ctrl(2);
            out = bicycleDynamics([Theta;delta;v;throttle]);
            dydt = [out; err_throttle;err_theta];
         else
            v = input(4);
            err_throttle = input(5);
            err_theta = input(6);
            error = ang-Theta;
            vel_ctrl = velocity_control(vel_setpoint,v,err_throttle);
            steer_ctrl = steering_control(error,err_theta);
            throttle = vel_ctrl(1);
            err_throttle = vel_ctrl(2);
            delta = steer_ctrl(1);
            err_theta = steer_ctrl(2);
            out = bicycleDynamics([Theta;delta;v;throttle]);
            dydt = [out; err_throttle;err_theta];
        end
    end
end
%end