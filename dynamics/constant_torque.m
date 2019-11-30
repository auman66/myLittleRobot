function tau = constant_torque(~,t,q,qd,tor_vect)
    %Torque function for constant torque
    %tor_vect is an Nx1 vector where N is the number of robot joints.
    %Each entry in tor_vect represents the constant torque for the
    %cooresponding joint.
    %t, q, and qd are not used
    
    tau = tor_vect;
end

