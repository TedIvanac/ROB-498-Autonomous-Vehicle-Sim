function steer_ctrl = steering_control(error,error_integral)

persistent errorlist

if isempty(errorlist)
    errorlist = 0;
end

% Gain

Dgain = .4;
Pgain = 190;
Igain = 37;


%Derivative
errorlist(end+1) = error;
der_error = errorlist(end) - errorlist(end-1);

% PID
P = Pgain*error;
I = Igain*error_integral;
D = Dgain*der_error;

% PID = P+I+D;
PID = P+I+D;
steer_ctrl = [PID, error];
end