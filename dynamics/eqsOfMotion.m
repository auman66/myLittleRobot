function Dyno = eqsOfMotion(R,q,qd,frame)
% Finds matrices for equations of motion for Robot arm
% Dyno = eqsOfMotion(R,q,qd,frame)
% Finds matrix coefficients for the equation of motion for serial link 
% robot R at position q and velocity qd. Frame is either world frame or 
% end-effector frame. 
% 
% Equation of motion: B(q)*qdd + C(q,qd)*qd + Fv*qd + Fs*sgn(qd) + g(q) = tau - J'(q)*he
%  
% Options
% frame => frame for end effector force vector. ?ee? for end effector frame 
% or ?w? for world frame. Default is end effector frame 
% 
% Output:
% Map of equation of motion values
% B: Symetric joint space inetria matrix (NxN)
% C: Coriolis matrix (NxN)
% F: Viscous & Coulomb friction (1xN)
% G: gravitational vector (1xN)
% J: Jacobian Matrix (6xN). Frame based on input


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

