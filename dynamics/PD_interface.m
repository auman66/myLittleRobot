function [t_arr, q_arr] = PD_interface(R, xd_initial, xd_final, time, steps, P, D, pd_space)
%Given an initial and final position/orientation vector in operational
%space, this function will produce a trajectory to get form one to another,
%and then use a PD controller in joint space to perform the motion.

%R          - Robot Serial Link
%xd_initial - Initial operational space vector
%xd_final   - Final operational space vector
%time       - Total Time to get from initial to final point
%steps      - Number of steps to discretize trajectory into
%P          - Proportional Gain
%D          - Derivative Gain
%pd_space   - String specifying either operational space or joint space 

NUM_OF_JOINTS = length(R.config);

%These should be preinitialized for memory...
t_arr = [];
q_arr = [];
q_dot_arr = [];

if strcmp(pd_space,'PD_Joint')
    T_initial = rt2tr(rpy2r(xd_initial(4:6)), xd_initial(1:3)');
    T_final   = rt2tr(rpy2r(xd_final(4:6)), xd_final(1:3)');  

    q_dot0 = zeros(NUM_OF_JOINTS,1);

    q_des_array = R.jtraj(T_initial, T_final, steps, 'ikine', @R.ikunc);

    q = q_des_array(1, :);
    q_dot = q_dot0;
    for i = 1:length(q_des_array)
        q_des = q_des_array(i, :);
        [t_arr_curr, q_arr_curr, q_dot_arr_curr] = dyno_wrap(R, (1/steps)*time, pd_space, q, q_dot, q_des, P, D);

        t_arr = [t_arr; (i-1)*(1/steps)*time + t_arr_curr];
        q_arr = [q_arr; q_arr_curr];
        q_dot_arr = [q_dot_arr; q_dot_arr_curr];


        q = q_arr_curr(end, :);
        q_dot = q_dot_arr_curr(end, :);

    end
    
    
elseif strcmp(pd_space,'PD_Oper')
    T_initial = rt2tr(rpy2r(xd_initial(4:6)), xd_initial(1:3)');
    T_final   = rt2tr(rpy2r(xd_final(4:6)), xd_final(1:3)'); 
    
    T_traj = ctraj(T_initial, T_final, steps);
    x_des_array = [];
    for i = 1:steps
        x_des_array = [x_des_array;[transl(T_traj(:,:, i))', tr2rpy(T_traj(:,:, i))]];
    end
    
    q0 = R.ikunc(T_traj(:,:,1));
    q_dot0 = zeros(NUM_OF_JOINTS, 1);
    
    q = q0;
    q_dot = q_dot0;
    for i = 1:length(x_des_array)
        x_des = x_des_array(i, :);
        [t_arr_curr, q_arr_curr, q_dot_arr_curr] = dyno_wrap(R, (1/steps)*time, pd_space, q, q_dot, x_des, P, D);

        t_arr = [t_arr; (i-1)*(1/steps)*time + t_arr_curr];
        q_arr = [q_arr; q_arr_curr];
        q_dot_arr = [q_dot_arr; q_dot_arr_curr];


        q = q_arr_curr(end, :);
        q_dot = q_dot_arr_curr(end, :);
        
    end
    
     
end

end