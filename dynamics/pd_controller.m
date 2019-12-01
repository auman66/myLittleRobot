function tau = pd_controller(~,t,q,qd,qstar,P,D)
    %Torque function for PD Controller
    %qstar is error term
    %P is Proportional Term
    %D is Derivative Term 
    %t is not used
    tau = P*(qstar-q) + D*qd;
end

