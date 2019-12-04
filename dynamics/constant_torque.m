function tau = constant_torque(~,t,q,qd,tor_vect)
% Torque function for constant torque
% tau = constant_torque(~,t,q,qd,tor_vect)
% Returns an Nx1 torque vector where N is the number of robot joints. The 
% torque vecotr is received from the tor_vect input. Each entry in tor_vect 
% represents the constant torque for the corresponding joint.
% t, q, and qd are not used
    
    tau = tor_vect;
end

