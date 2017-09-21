#PID_Contoller:


Github repository: https://github.com/prkay88/PID_Controller.git

1. Clone or download the git repository from the above link.
2. Open it with MATLAB.
3. Run the runrobotarm function with its parameter. 
   
    runrobotarm(time_max, link1_len,link2_len,link1_mass,link2_mass, torque_limit,joint1_init_angle,joint2_init_angle, joint1_desired_angles,joint2_desired_angles)
     
    Meaning of Parameters for runrobotarm function: 

    time = max - How many simulated seconds to run the simulation.

    link1_len - length of the first link in meters.

    link2 _len - length of the second link in meters.

    link1_mass - mass of the first link in Kg.

    link2_mass - mass of the second link in Kg.

    torque_limit - maximum angular force that can be applied by the actuators.

    joint1_init_angle - initial angle of the first joint.

    joint2_init_angle - initial angle of the second joint.

    joint1_desired_angles - a two column matrix of time, joint anglepairs for the first joint.

    joint2_desired_angles - a two column matrix of time, joint angle pairs for the first joint.

    For instance,  
    You can run the runrobotarm function like below:
    runrobotarm(10,1,1,1,1,500,0.01,0,[0,pi:5 ,pi/2],[0,pi/2;5,pi])