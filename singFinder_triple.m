function [sings, doubles, triples] = singFinder(robot)
%Finds single joint singularities for robot
%   Detailed explanation goes here

    sings = [];
    doubles = [];
    triples = [];
    alphas = robot.alpha;
    alphas(alphas <= 0) = [];
    pi_list = -1*pi():pi()/24:pi();
    pi_list = pi_list(2:end);

    RN = robot.n;
    smallP = pi/17;

    for i=1:RN
        Q = smallP*ones(1,RN);
        for j=pi_list
            Q(i) = j;
            J = jacobn(robot,Q);
            [r, jb] = rref(J);
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
    if isempty(doubles)
        d = [0 0 0 0];
    else
        d = doubles;
    end
    pi_list = -1*pi():pi()/4:pi();
    pi_list = pi_list(2:end);
    for i=1:RN
        for j=i+1:RN
            for k=j+1:RN
                Q = smallP*ones(1,RN);
                for ii=pi_list
                    if ~ismember([i ii],s,'rows')
                        for jj=pi_list
                            if ~ismember([j jj],s,'rows') && ~ismember([i ii j jj],d,'rows')
                                for kk=pi_list
                                    if ~ismember([k kk],s,'rows') && ~ismember([i ii k kk],d,'rows') && ~ismember([j jj k kk],d,'rows') 
                                        Q(i) = ii;
                                        Q(j) = jj;
                                        Q(k) = kk;
                                        J = jacobn(robot,Q);
                                        [r, jb] = rref(J);
                                        depcols = setdiff( 1:numcols(J), jb);
                                        if depcols > 0
                                            triples = [triples;i ii j jj k kk];
                                        end
                                    end
                                end
                            end
                        end
                    end
                end
            end
        end
    end
end

