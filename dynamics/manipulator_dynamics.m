function Dyno = manipulator_dynamics(R,q,qd,qdd)
%Manipulator dynamics
%   find the equations of motion for a manipulator and
%   plot joints positions and velocities

% R = SerialLink Robot
% q = Nx1 joint configuration matrix
% qd = Nx1 joint velocity matrix

% B(q)*qdd + C(q,qd)*qd + Fv*qd + Fs*sgn(qd) + g(q) = tau - J'(q)*he
B = R.inertia(q); %symetric joint space inetria matrix (NxN)
C = R.coriolis(q,qd); %Coriolis matrix (NxN)
F = R.friction(qd); %Viscous & Coulomb friction (1xN)
g = R.gravload(q); %gravitational vector (1xN)
J = R.jacobO(q); %Jacobian in world frame

keySet = {'B','C','F','G','J'};
valueSet = {B,C,F,g,J};
Dyno = containers.Map(keySet,valueSet);

end

