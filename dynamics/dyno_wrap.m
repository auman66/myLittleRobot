function [t, q, qd] = dyno_wrap(robot, t, torqfun, q0, qd0, varargin)
% Wrapper function for forward dynamics with robotic toolbox
%  
% [T,Q,QD] = dyno_wrap(robot, t, torqfun, q0, qd0, varargin) returns the 
% results from the robotics toolbox fdyn function. This wrapper makes it 
% easier to use with the MLR GUI. 
% 
% See the fdyn function in the robotics toolbox for more information.
% 
% Inputs
% - robot 		Serial Link Object
% - t 		Time  interval 0 to t
% - torqfun	Torque function
% - q0  	Initial joint position
% - qd0 	Initial joint velocities
% - varagin	Arguments passed to the torque function
% 
% Options
% - torqfun can be a pointer to a torque function, the name of a torque 
%   function, or a predefined torque function.
% - torqfun = ?constant? will use a constant torque function
%   - varagin => T is a 1xN vector of torques and N is the number of joints
% - torqfun = ?PD? will use a constant torque function
%   - varagin => P is Proportional Gain
%   - varagin => D is Derivative Gain


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
    elseif strcmp(torqfun,'PD_Joint')
        %Need to convert from world space to joint space for qstar
        [t,q,qd] = robot.fdyn(t,@joint_pd_controller,q0,qd0,varargin{:});
    elseif strcmp(torqfun, 'PD_Oper')
        [t,q,qd] = robot.fdyn(t,@oper_pd_controller,q0,qd0,varargin{:});
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


