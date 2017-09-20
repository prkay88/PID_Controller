 function control_forces = PIDController( current_time, joint1_angle_setpoint, joint2_angle_setpoint, joint1_measured_angle, joint2_measured_angle, error1_dot, error2_dot )
z =
%% LAST KNOWN ERRORS Ax *yND TIME 
persistent lastError;
persistent lastTime;
persistent totalIntegral;


%% If first time running through PID
if(isempty(lastTime) && isempty(lastError))
    lastError = [0;0];
    lastTime = 0.01;
    totalIntegral = [0;0];
end

%% Instantiate constants
kp = 1;
kd = 1;
ki = 1;


%% CALCULATING PID FOR BOTH JOINTS
% f(x) = Kp(desiredYaw - actualYaw) + Kd(deltaError/deltaTime) + integral[(errordt)Ki]

dTime = current_time - lastTime;

propTerm = [joint1_angle_setpoint - joint1_measured_angle ;
            joint2_angle_setpoint - joint2_measured_angle ];
devTerm = (propTerm - lastError)./ dTime;        
intTerm = dTime .* propTerm;
 
 totalIntegral = totalIntegral + intTerm;
 
 % calculating by using the derivative term given from the parameter of the
 % function
 %control_forces = kp .* propTerm + kd .* [error1_dot;error2_dot] + ki .* totalIntegral;
 control_forces = kp .* propTerm + kd .* devTerm + ki .* totalIntegral;
 % Calculating by using the the derivative term that we calacuated on our
 % own. To get more than 93/100 in this assignment we are supposed to
 % create a derivative term on our own. 
 % Formula for control force using our own derivative term is : 
 % control_forces = kp .* propTerm + kd .* devTerm + ki .* totalIntegral;


%% Setup function for next calculation
lastError = propTerm;
lastTime = current_time;

end