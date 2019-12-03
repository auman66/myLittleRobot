function [sings, doubles] = singFinder(robot)
%Finds single joint singularities for robot
%   Detailed explanation goes here

    sings = [];
    doubles = [];
    pi_list = -1*pi():pi()/24:pi();
    pi_list = pi_list(2:end);

    RN = robot.n;
    smallP = pi/17;

    for i=1:RN
        Q = smallP*ones(1,RN);
        for j=pi_list
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
    pi_list = -1*pi():pi()/12:pi();
    pi_list = pi_list(2:end);
    for i=1:RN
        for j=i+1:RN
            Q = smallP*ones(1,RN);
            for ii=pi_list
                if ~ismember([i ii],s,'rows')
                    for jj=pi_list
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

