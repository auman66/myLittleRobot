function [t, q, qd] = dyno_wrap(robot, t, torqfun, q0, qd0, varargin)
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
    if strcmp(torqfun,'constant')
        t_fun = @constant_torque;
        tau = varargin{1};
        %make sure input vector is correct size for constant_torque
        if length(tau) > n
            tau = tau(1:n);
        elseif length(tau) < n
            tau = [tau, zeros(1,n - length(tau))];
        end
        [t,q,qd] = robot.fdyn(t,t_fun,q0,qd0,tau);
    elseif strcmp(torqfun,'PD')
        %Need to convert from world space to joint space for qstar
        [t,q,qd] = robot.fdyn(t,@pd_controller,q0,qd0,varargin{:});
    elseif ~isa(torqfun,'function_handle')
        %clean up the input to remove any leading or trailing spaces
        temp_fun = deblank(torqfun);  %remove trailing spaces
        temp_flipped = temp_fun(end:-1:1);  %flip the string
        temp_flipped = deblank(temp_flipped);  %remove trailing spaces of the flipped string
        temp_fun = temp_flipped(end:-1:1);  %flip the string back
        temp_fun = char(torqfun); %convert to a character array if it isn't already
        t_fun = str2func(temp_fun);
        [t,q,qd] = robot.fdyn(t,t_fun,q0,qd0,varargin{:});
    else
        [t,q,qd] = robot.fdyn(t,torqfun,q0,qd0,varargin{:});
    end
   



end

