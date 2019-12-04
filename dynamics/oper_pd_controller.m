function tau = oper_pd_controller(R,t,q,q_dot,x_desired,P,D)
    %Torque function for PD Controller called by fdyn to create torque
    %values
    %R     Serial Link Object
    %q     Current joint configuration of robot
    %q_dot Current joint velocities of robot
    %
    %NOTE: R, q, and q_dot are passed in by fdyn
    %
    %x_desired  Vector of desired joint space variables
    %P     Proportional Term
    %D     Derivative Term 
    %t     Not used
    
    T_curr = R.fkine(q);
    x_curr = [transl(T_curr)', tr2rpy(T_curr)];
   
    error = x_desired - x_curr;
    P_mat = P*eye(length(error));
    D_mat = D*eye(length(error));
    
    Ja = R.jacob0(q, 'rpy');
    
    tau = (Ja'*(P_mat*error' - D_mat*Ja*q_dot'))' + R.gravload(q);

end