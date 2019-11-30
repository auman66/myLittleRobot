function [t, q, qd] = dyno_wrap(robot, t1, torqfun, q0, qd0, varargin)
    %Forward Dynamics Wrapper Function
    
    
    %Need to make sure there are robot values for m, r, I, G, Jm

    %Need to determine direction of gravity and friction on/off
    %     R.nofriction;
    %     R.gravity = [0 0 -1];n = robot.n;
    n = robot.n;
    if nargin == 2
        torqfun = 0;
        q0 = zeros(1,n);
        qd0 = zeros(1,n);
    elseif nargin == 3
        q0 = zeros(1,n);
        qd0 = zeros(1,n);
    elseif nargin == 4
        qd0 = zeros(1,n);
    end
    %set contant torque function
    if torqfun == 'constant'
        t_fun = @constant_torque;
        tau = varargin{1};
        %make sure input vector is correct size for constant_torque
        if length(tau) > n
            tau = tau(1:n);
        elseif length(tau) < n
            tau = [tau, zeros(1,n - length(tau))];
        end
        [t,q,qd] = robot.fdyn(t1,t_fun,q0,qd0,tau);
            
    else
        [t,q,qd] = robot.fdyn(t1,torqfun,q0,qd0,varargin{:});
    end
   



end

