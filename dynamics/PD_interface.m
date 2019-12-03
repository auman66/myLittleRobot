function [t_arr, q_arr] = PD_wrapper(R, xd_initial, xd_final, time, steps, P, D, pd_space)
%Given an initial and final position/orientation vector in operational
%space, this function will produce a trajectory to get form one to another,
%and then use a PD controller in joint space to perform the motion.

%R          - Robot Serial Link
%xd_initial - Initial operational space vector
%xd_final   - Final operational space vector
%time       - Total Time to get from initial to final point
%steps      - Number of steps to discretize trajectory into
%P          - Proportional Gain Matrix
%D          - Derivative Gain Matrix
%pd_space   - String specifying either 



T_initial = rt2tr(rpy2r(xd_initial(4:6)), xd_initial(1:3)');
T_final   = rt2tr(rpy2r(xd_final(4:6)), xd_final(1:3)');  

q_dot0 = zeros(3,1);

q_des_array = R.jtraj(T_initial, T_final, steps, 'ikine', @R.ikunc);

%These should be preinitialized for memory...
t_arr = [];
q_arr = [];
q_dot_arr = [];

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

end