function [outputArg1,outputArg2] = manipulator_dynamics(robot,q_0,qd_0,qdd_0)
%Manipulator dynamics
%   find the equations of motion for a manipulator and
%   plot joints positions and velocities

% R = SerialLink Robot
% q = Nx1 joint configuration matrix

% B(q)*qdd + C(q,qd)*qd + Fv*qd + Fs*sgn(qd) + g(q) = tau - J'(q)*he
B = robot.inertia(q); %symetric joint space inetria matrix (NxN)
C = robot.coriolis(q,qd); %Coriolis matrix (NxN)
F = robot.friction(qd); %Viscous & Coulomb friction (1xN)
g = robot.gravload(q);







end

