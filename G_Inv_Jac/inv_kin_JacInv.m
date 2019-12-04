function [q] = inv_kin_JacInv(R, xd)
%inv_kin_JacInv: Uses iterative Jacobian Pseudo-Inverse Algorithm
%to compute joint space values.
%
%R - SerialLink Object
%xd - [x,y,z,phi, psi, theta]
%     
%q - solution to Inv Kin Problem

NUM_OF_JOINTS = length(R.config);

q_arr = zeros(NUM_OF_JOINTS, length(xd)); %Preallocate memory for storing joint space solutions

q_arr(:,1) = rand(1, NUM_OF_JOINTS); %Set initial seed to rand vals (zeros are typically a singularity...)

%Gain Matrix 
K=100*eye(6,6);

MaxIter = 100;
T = 0.001; %Integration Interval

xd_repeat = repmat(xd', 1, MaxIter);

Rd = rpy2r(xd(4:6));

for i = 1:MaxIter
    %Use Forward Kinematics to get Xe
     Te = R.fkine(q_arr(:, i));
     xyz_e = transl(Te);    %Positional Components
     phi = tr2rpy(t2r(Te)); %Rotational Components
    
    %Use quaternions to be cool, and most importantly avoid non-unique vals
    Re = tr2rt(Te);
    quat_d = rotm2quat(Rd);
    quat_e = rotm2quat(Re);

    delta_e = quat_e(1)*quat_d(2:end) - quat_d(1)*quat_e(2:end);
    
    xe(:, i) = [xyz_e', phi];
    
    J = R.jacob0(q_arr(:, i));
    
    error(:,i)= xd_repeat(:,i) - xe(:,i);
    error(4:6, i) = delta_e;
    
    q_dot = pinv(J)*(K*error(:,i));
    q_arr(:,i+1)= q_arr(:,i)+q_dot*T;
end

q = q_arr(:, end);

end

