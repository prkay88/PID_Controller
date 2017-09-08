% Enter your solution to the assignment as the body of this function. Do not modify the code outside this file.

function control_forces = PIDController( current_time, joint1_angle_setpoint, joint2_angle_setpoint, joint1_measured_angle, joint2_measured_angle, error1_dot, error2_dot )

%LAST KNOWN ERRORS AND TIME
persistent lastError1; 
persistent lastError2;
persistent lastTime;

torque1 = 0;
torque2 = 0;

%If first time running through PID
if(isempty(lastError1) && isempty(lastError2) && isempty(lastTime))
    lastError1 = 0;
    lastError2 = 0;
    lastTime = 0.01;
end

%instantiate constants
kp = 1;
kd = 1;
ki = 1;

%Find Delta Time
dTime = current_time - lastTime;

%Instantiate known steps
propTerm1 = 0;
propTerm2 = 0;
divTerm1 = 0;
divTirm2 = 0;
f1 = 0;
f2 = 0;
   
% f(x) = Kp(desiredYaw - actualYaw) + Kd(deltaError/deltaTime) + integral[(errordt)Ki]
%-----------------------------------------------------------

%JOINT1
%-----------------------------------------------------------
%Proportional  Term:    
propTerm1 = (joint1_angle_setpoint - joint1_measured_angle);    %Working.... ish
%Derivitive Term:
divTerm1 = (propTerm1 -lastError1) / (dTime);                   %?
%Integral Term:
intTerm1 = 0;                                                   %working on calculation

torque1 = (kp * propTerm1) + (kd * divTerm1)% + (ki * f1)    %calculate

%-----------------------------------------------------------

%JOINT2
%-----------------------------------------------------------
%Proportional  Term:    
propTerm2 = (joint2_angle_setpoint - joint2_measured_angle);    %Working... ish?
%Derivitive Term:
divTerm2 = (propTerm2 -lastError2) / (dTime);                   %?
%Integral Term:
intTerm2 = 0;                                                   %working on calculation

torque2 = 0 % (kp * propTerm2)% + (kd * errorTime2)% + (ki * f2) 

%-----------------------------------------------------------

%integral(fun,0,1)

%Store in vector
control_forces = [torque1; torque2];

%Setup function for next calculation
%-----------------------------------------------------------
lastError1 = propTerm1;
lastError2 = propTerm2;
lastTime = current_time;

end
