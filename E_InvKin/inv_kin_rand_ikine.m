function [sol_arr] = inv_kin_rand_ikine(R, T_des, q0,m);

MaxIter = 50;
num_of_joint_vars = length(R.config);

inv_q = zeros(MaxIter, num_of_joint_vars);
inv_q(1, 1:num_of_joint_vars) = q0;

inv_q(1, 1:num_of_joint_vars) = R.ikine(T_des, rand(1,num_of_joint_vars).*q0, m);
for i = 2:MaxIter
    inv_q(i, 1:num_of_joint_vars) = R.ikine(T_des, -0.5+rand(1,num_of_joint_vars).*inv_q(i, 1:num_of_joint_vars), m);
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