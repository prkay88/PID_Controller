% Enter your solution to the assignment as the body of this function. Do not modify the code outside this file.

function control_forces = PIDController( current_time, joint1_angle_setpoint, joint2_angle_setpoint, joint1_measured_angle, joint2_measured_angle, error1_dot, error2_dot )

%% CONSTANTS
kp = 170;   %Adjust the proportional term to get fast response (GO BIG)
ki = .18;     %Adjust integeral term to get graph close to 0 (will cause overshoot)
kd = 35;     %Adjust Derivative term to lower the overshoot

%% PERSISTANT VARIABLES
persistent last_time;
persistent last_error;
persistent total_integral;
persistent last_Derivative;

if(isempty(last_time))
   last_time = 0;
   last_error = [0;0];
   total_integral = [0;0];
   last_Derivative = [0;0];
end

%% TIME
delta_time = current_time - last_time;

%% CURRENT ERROR
current_error = [joint1_angle_setpoint - joint1_measured_angle;
    joint2_angle_setpoint - joint2_measured_angle];

%% ITERATE INTEGRAL
total_integral(1) = (total_integral(1) + current_error(1));
total_integral(2) = (total_integral(2) + current_error(2));

%% CALCULATE DERIVATIVE
derivative = [0;0];
if(delta_time ~= 0)
    derivative = [(current_error(1) - last_error(1)) / delta_time;
        (current_error(2) - last_error(2)) / delta_time];
else
    derivative = [last_Derivative(1); last_Derivative(2)];

end

%% OUTPUT
Pout = [(kp * current_error(1));
    (kp * current_error(2))];

Iout = [(ki * total_integral(1));
    (ki * total_integral(2))];

Dout = [(kd * derivative(1));
    (kd * derivative(2))];

%% CALCULATE TORQUES
torque1 = Pout(1) + Iout(1) + Dout(1);
torque2 = Pout(2) + Iout(2) + Dout(2);


last_error = current_error;
last_time = current_time;
last_Derivative = derivative;

control_forces = [torque1;torque2];


end
