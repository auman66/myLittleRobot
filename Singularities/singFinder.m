function [sings, doubles] = singFinder(robot,div)
%Finds single and doulbe joint singularities for robot
%   
% Single Joint Singularity: A single joint singularity refferes to when a
% single joint in a specific orientation cauases a singularity







    if ~exist('div','var')
        % div parameter does not exist, so default it to something
        div = 48;
    end

    sings = [];
    doubles = [];
    

    RN = robot.n;
    smallP = pi/17;

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
    if isempty(sings)
        s = [0 0];
    else
        s = sings;
    end

    for i=1:RN
        for j=i+1:RN
            Q = smallP*ones(1,RN);
            q_list_i = qLister(robot.links(i),ceil(div/2));
            for ii=q_list_i
                if ~ismember([i ii],s,'rows')
                    q_list_j = qLister(robot.links(j),ceil(div/2));
                    for jj=q_list_j
                        if ~ismember([j jj],s,'rows')
                            Q(i) = ii;
                            Q(j) = jj;
                            J = jacobn(robot,Q);
                            [r, jb] = rref(J);
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

