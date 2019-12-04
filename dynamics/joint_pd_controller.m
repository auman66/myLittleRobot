function tau = joint_pd_controller(R,t,q,q_dot,q_desired,P,D)
    %Torque function for PD Controller
    %P is Proportional Term
    %D is Derivative Term 
    %t is not used
   
    error = q_desired - q;
    
    P_mat = P*eye(length(error));
    D_mat = D*eye(length(error));
    
    tau = (P_mat*error' - D_mat*q_dot')' + R.gravload(q);

end

