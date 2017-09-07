% Enter your solution to the assignment as the body of this function. Do not modify the code outside this file.

function control_forces = PIDController( current_time, joint1_angle_setpoint, joint2_angle_setpoint, joint1_measured_angle, joint2_measured_angle, error1_dot, error2_dot )

persistent lastError1; 
persistent lastError2;
persistent lastTime;

torque1 = 0;
torque2 = 0;

if(isempty(lastError1) && isempty(lastError2) && isempty(lastTime))
    lastError1 = 0;
    lastError2 = 0;
    lastTime = 0.01;
end

kp = 1;
kd = 1;
ki = 1;

%deltaTime
dTime = current_time - lastTime

propTerm1 = 0;
propTerm2 = 0;
divTerm1 = 0;
errorTime2 = 0;
f1 = 0;
f2 = 0;
   
% f(x) = Kp(desiredYaw - actualYaw) + Kd(deltaError/deltaTime) + integral[(errordt)Ki]
   
%-----------------------------------------------------------

%JOINT1
   
%Proportional  Term:    
propTerm1 = (joint1_angle_setpoint - joint1_measured_angle);
%Derivitive Term:
divTerm1 = (propTerm1 -lastError1) / (dTime);
%Integral Term:
intTerm1 = 0;  %working on calculation

%-----------------------------------------------------------

%JOINT2
%Proportional  Term:    
propTerm2 = (joint2_angle_setpoint - joint2_measured_angle);
%Derivitive Term:
divTerm1 = (propTerm2 -lastError2) / (dTime);
%Integral Term:
intTerm2 = 0;  %working on calculation

%integral(fun,0,1)

%Calculate Torque & Fill Vector
%-----------------------------------------------------------
% torque1, torque2 paired with joint1, joint2
torque1 = (kp * propTerm1)% + (kd * errorTime1)% + (ki * f1)
torque2 = 0 % (kp * propTerm2)% + (kd * errorTime2)% + (ki * f2) 



control_forces = [torque1; torque2];

%Setup function for next calculation
%-----------------------------------------------------------
lastError1 = propTerm1
lastError2 = propTerm2
lastTime = current_time

end
