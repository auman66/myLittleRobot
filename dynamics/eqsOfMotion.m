function Dyno = eqsOfMotion(R,q,qd,frame)
%Manipulator dynamics: Find the equations of motion for a manipulator
%   Returns values based on equation of motion:
%   B(q)*qdd + C(q,qd)*qd + Fv*qd + Fs*sgn(qd) + g(q) = tau - J'(q)*he

% Inputs:
%   R = SerialLink Robot
%   q = Nx1 joint configuration matrix
%   qd = Nx1 joint velocity matrix
%   frame = frame for end effector force vector. Default is end effector frame 
%           Options: 'ee' (end effector) or 'w' (world)

%Outputs:
%   See below
%   Note: Fv*qd + Fs*sgn(qd) is combined into F term


B = R.inertia(q); %symetric joint space inetria matrix (NxN)
C = R.coriolis(q,qd); %Coriolis matrix (NxN)
F = R.friction(qd); %Viscous & Coulomb friction (1xN)
G = R.gravload(q); %gravitational vector (1xN)

if ~exist('frame','var')
    % frame parameter does not exist, so default it to end effector
    frame = 'ee';
end


if strcmp(frame,'w')
    J = R.jacobO(q); %Jacobian in world frame
else
    if ~strcmp(frame,'ee')
        warning('Frame not defined as end-effector (ee) or world (w). Using end-effector frame')
    end
    J = R.jacobn(q); %Jacobian in end effector frame
end

keySet = {'B','C','F','G','J'};
valueSet = {B,C,F,G,J};
Dyno = containers.Map(keySet,valueSet);

end

