
function [cost] = CostFunction(R, x, m, T_des)
    %Compute Error based off the sum of the square error in operational
    %space
    
    %Find homogeneous matrix based off current joint params
    T_e = R.fkine(x);
    rpy_e = tr2rpy(T_e); %Get the RPY angles
    xyz_e = T_e(1:3, 4)';%Get the end-effector position
    xe = [xyz_e, rpy_e];
    xe = xe.*m';         %Apply masking
    
    %Perform the same operations on the desired end-effector homogeneous
    %matrix
    rpy_e = tr2rpy(T_des);
    xyz_e = T_des(1:3, 4)';
    x_des = [xyz_e, rpy_e];
    x_des = x_des.*m'; 
    
    %Compte the sum of the square errors...
    cost = sum((x_des - xe).^2);
end