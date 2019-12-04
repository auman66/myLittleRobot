function tau = joint_pd_controller(R,t,q,q_dot,x_desired,P,D)
    %Torque function for PD Controller
    %P is Proportional Term
    %D is Derivative Term 
    %t is not used
    
    T_curr = R.fkine(q);
    x_curr = [transl(T_curr)', tr2rpy(T_curr)];
   
    error = x_desired - x_curr;
    P_mat = P*eye(length(error));
    D_mat = D*eye(length(error));
    
    Ja = R.jacob0(q, 'rpy');
    
    tau = (Ja'*(P_mat*error' - D_mat*Ja*q_dot'))' + R.gravload(q);

end