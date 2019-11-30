function [q] = inv_kin_JacInv(R, xd, xd_dot, m, T)
%inv_kin_JacInv: Uses iterative Jacobian Pseudo-Inverse Algorithm
%to compute joint space values.
%
%R - SerialLink Object
%xd - Time Series vector of desired Operational Space values of the form
%     [x,y,z,phi, psi, theta]
%xd_dot - Derivative of xd
%m - masking vector for operational space 
%T - Sampling Period for xd/xd_dot
%
%q - time series vector of joint space solutions

NUM_OF_JOINTS = length(R.config);

q = zeros(NUM_OF_JOINTS, length(xd)); %Preallocate memory for storing joint space solutions

q(:,1) = rand(1, NUM_OF_JOINTS); %Set initial seed to rand vals (zeros are typically a singularity...)

%Gain Matrix 
K=100*eye(3,3);

for i = 1:length(xd)
    %Use Forward Kinematics to get Xe
    Te = R.fkine(q(:, i), m);
    xyz_e = transl(Te);    %Positional Components
    phi = tr2rpy(t2r(Te)); %Rotational Components
    
    %Removes unused operational space components (due to redudancy)
    xe_temp = [xyz_e', phi];
    xe(:, i) = xe_temp(logical(m));
    
    %Removes unused operational space rows (due to redudancy)
    Ja_temp = R.jacob0(q(:, i));
    Ja = [Ja_temp(1:2, :); Ja_temp(end, :)];
    
    error(:,i)= xd(:,i) - xe(:,i);
    size(error)
    
    q_dot = pinv(Ja)*(xd_dot(:, i)+K*error(:,i));
    q(:,i+1)= q(:,i)+q_dot*T;
end

figure();
plot(xe')
figure();
plot(xd')
figure()
plot(xd'-xe')
end

