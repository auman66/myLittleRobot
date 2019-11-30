function [sol_arr] = inv_kin_rand_ikine(R, T_des, q0,m);

inv_q = zeros(50, 3);
inv_q(1, 1:3) = q0;
inv_q(1, 1:3) = R.ikine(T_des, rand(1,3).*q0, m);
for i = 2:50
    inv_q(i, 1:3) = R.ikine(T_des, -0.5+rand(1,3).*inv_q(i, 1:3), m);
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