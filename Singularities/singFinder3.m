function [sings, doubles, triples] = singFinder3(robot,div)
%Finds single joint singularities for robot
%   Detailed explanation goes here
    
    if ~exist('div','var')
        % div parameter does not exist, so default it to something
        div = 48;
    end

    [sings, doubles] = singFinder(robot,div);
    triples = [];
    
    RN = robot.n;
    smallP = pi/17;

    if isempty(sings)
        s = [0 0];
    else
        s = sings;
    end
    
    if isempty(doubles)
        d = [0 0 0 0];
    else
        d = doubles;
    end
    

    for i=1:RN
        for j=i+1:RN
            for k=j+1:RN
                Q = smallP*ones(1,RN);
                i_list = qLister(robot.links(i),ceil(div/4));
                for ii=i_list
                    if ~ismember([i ii],s,'rows')
                        j_list = qLister(robot.links(j),ceil(div/4));
                        for jj=j_list
                            if ~ismember([j jj],s,'rows') && ~ismember([i ii j jj],d,'rows')
                                k_list = qLister(robot.links(k),ceil(div/4));
                                for kk=k_list
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

