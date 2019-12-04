function [sol_arr] = inv_kin_rand_ikine(R, T_des, q0);
%Given a homogeneous transformation matrix corresponging to a desired
%end-efector pose and orientation, this function will return a
%corresponding joint space vector, and possibly multiple solutions if they
%can be found
%R     SerialLink Object
%T_des homogeneous transformation matrix corresponging to a desired end-efector pose and orientation
%q0 - and initial guess to try 
%
%sol_arr - solutions to the inverse kinematics problem. If multiple
%solutions are found, it will return them also
MaxIter = 50;
num_of_joint_vars = length(R.config);

inv_q = zeros(MaxIter, num_of_joint_vars);
inv_q(1, 1:num_of_joint_vars) = q0;

inv_q(1, 1:num_of_joint_vars) = R.ikunc(T_des, rand(1,num_of_joint_vars).*q0);
for i = 2:MaxIter
    inv_q(i, 1:num_of_joint_vars) = R.ikunc(T_des, -0.5+rand(1,num_of_joint_vars).*inv_q(i, 1:num_of_joint_vars));
end

    revolute_mask = (R.config == 'R'); %Logical array for joints that are
                                       %revolute
    for i = 1:length(revolute_mask)
        if (revolute_mask(i))
            inv_q(:, i) = wrapToPi(inv_q(:, i));
        end
    end

sol_arr = uniquetol(inv_q, 1E-6, 'ByRows', true);

end