function [sings, doubles] = singFinder(robot,div)
%Finds single and doulbe joint singularities for robot
%   
% Single Joint Singularity: 
%   When a single joint in a specific orientation cauases a singularity.
% Double Joint Singularity:
%   When a pair of joints in a specific orientation casuse a singularity.

% Output:
% Singles => [joint # , angle of singularity]
% Doubles => [first joint, first angle, second joint, second angle] 

% Singularities are found by trying a large number of joint configurations
% and testing to see if each one is a singularity. Specifically, this
% functions holds all the joint angles constant and then varies one or two
% joints at a time in order to find singularities

% WARNING: this function will not find all robot singularities. More may
% exist!!!

% Output:
% if only one value, will return single singularities
% if two values will return single and double singularities


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

