function [q_list] = qLister(joint,div)
% Returns a list of joint parameters to test in singularity finder
% functions

% Revolute Joint: first looks at range from -2pi to 2pi and then reduces to
% be within joint limits, or leaves at -2pi to 2pi if none is available
%
% Prismatic Joint: finds values within range limit. Sets to -1 to 1 if not
% limit is available. 


    qlim = joint.qlim;
    
    %revolute joint
    if joint.RP == 'R'
        step = 4*pi()/div;
        q_list = -2*pi():step:2*pi();
        %automatically include some typical singularity angels in case the
        %div misses them
        set_angles = -2*pi:pi/2:2*pi;
        q_list = unique([q_list set_angles]);
        if ~isempty(qlim)
            q_list = q_list(q_list>qlim(1));
            q_list = q_list(q_list<qlim(2));
            q_list = [q_list qlim];
            q_list = sort(unique(q_list));
        end
        
    %prismatic joint
    elseif joint.RP == 'P'
        if isempty(qlim)
            qlim = [-1 1];
        end
        step = (qlim(2)-qlim(1))/div;
        q_list = qlim(1):step:qlim(2);
        %make sure 0, a typical singularity value, is on the list
        q_list = [q_list 0];
        q_list = sort(unique(q_list));
    else
        q_list = [0];
    end
end

