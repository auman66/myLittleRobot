function tau = pd_controller(R,t,q,q_dot,q_desired,P,D)
    %Torque function for PD Controller
    %qstar is error term
    %P is Proportional Term
    %D is Derivative Term 
    %t is not used
   
    error = q_desired - q;
    
    tau = (P*error' - D*q_dot')' + R.gravload(q);

end

