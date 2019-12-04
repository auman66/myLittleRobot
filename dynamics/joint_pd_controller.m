function tau = joint_pd_controller(R,t,q,q_dot,q_desired,P,D)
    %Torque function for PD Controller called by fdyn to create torque
    %values
    %R     Serial Link Object
    %q     Current joint configuration of robot
    %q_dot Current joint velocities of robot
    %
    %NOTE: R, q, and q_dot are passed in by fdyn
    %
    %q_desired  Vector of desired joint space variables
    %P     Proportional Term
    %D     Derivative Term 
    %t     Not used
    
    %tau   generalized force vector for robot joints
   
    error = q_desired - q;
    
    P_mat = P*eye(length(error));
    D_mat = D*eye(length(error));
    
    tau = (P_mat*error' - D_mat*q_dot')' + R.gravload(q);

end

