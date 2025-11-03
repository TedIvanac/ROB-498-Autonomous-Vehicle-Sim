function vel_ctrl = velocity_control(target_vel,current_velocity,error_integral)

% Gain
Pgain = 2000;


% Proportional
error = target_vel - current_velocity;


% PID
P = Pgain*error;


% PID = P+I+D;
vel_ctrl = [P, error]; 
end