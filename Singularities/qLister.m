function [q_list] = qLister(joint,div)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
    qlim = joint.qlim;
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
    elseif joint.RP == 'P'
        if isempty(qlim)
            qlim = [-1 1];
        end
        step = (qlim(2)-qlim(1))/div;
        q_list = qlim(1):step:qlim(2);
        q_list = [q_list 0];
        q_list = sort(unique(q_list));
    else
        q_list = [0];
    end
end

