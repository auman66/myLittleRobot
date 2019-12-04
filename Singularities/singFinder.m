function [sings, doubles] = singFinder(robot,div)
% Finds single joint and coupled joint singularities for a robot
%
% [sings, couples] = singFinder(robot,div) outputs two matrices that list 
% single and coupled singularities for robot. sings is a 2xN matrix, where 
% N is the number of discovered single joint singularities. The first 
% column is the joint index and the second column is the joint angle that 
% causes a singularity. couples is a 4xN matrix, where N is the number of 
% discovered coupled joint singularities. The first column is the joint 
% index of the first joint, the second column is the joint angle of the 
% first joint, the third column is the index of the second joint, and the 
% fourth column is the joint angle of the second joint. 
% 
% Options
% - div is an optional parameter that denotes the number of steps when 
%   finding possible singularities. The default value is 96.
% 
% Notes
% - Singularities are found by trying a large number of joint 
%   configurations and testing to see if each one is a singularity. 
%   Specifically, this functions holds all the joint angles constant and 
%   then varies one or two joints at a time in order to find singularities.
% - This function will not find all robot singularities. More may exist.



    %set default value for div
    if ~exist('div','var')
        % div parameter does not exist, so default it to 96
        div = 96;
    end

    sings = [];
    doubles = [];
    RN = robot.n;
    
    %Choose joint angle that is unlikely to be a singularity
    smallP = pi/17;

    %find single singularities
    for i=1:RN
        Q = smallP*ones(1,RN);
        q_list = qLister(robot.links(i),div);
        for j=q_list
            Q(i) = j;
            J = jacobn(robot,Q);
            [~, jb] = rref(J);
            depcols = setdiff( 1:numcols(J), jb);
            if depcols > 0
                sings = [sings;i j];
            end
        end
    end
    %create 0s singularity array for next part, if none were found
    if isempty(sings)
        s = [0 0];
    else
        s = sings;
    end

    %find double singularities
    for i=1:RN
        for j=i+1:RN
            Q = smallP*ones(1,RN);
            q_list_i = qLister(robot.links(i),ceil(div/4));
            for ii=q_list_i
                %ignore sets where a single singularity exists
                if ~ismember([i ii],s,'rows')
                    q_list_j = qLister(robot.links(j),ceil(div/4));
                    for jj=q_list_j
                        %ignore sets where a single singularity exists
                        if ~ismember([j jj],s,'rows')
                            Q(i) = ii;
                            Q(j) = jj;
                            J = jacobn(robot,Q);
                            [~, jb] = rref(J);
                            depcols = setdiff( 1:numcols(J), jb);
                            if depcols > 0
                                doubles = [doubles;i ii j jj];
                            end
                        end
                    end
                end
            end
        end
    end
end

