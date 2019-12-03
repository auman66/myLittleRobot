function [sings, doubles, triples] = singFinder3(robot,div)
%Extends singFinder to also find triple singularities
%   
% Tripple Joint Singularity:
%   When a set of three joints in a specific orientation casuse a singularity.
% See singFinder for details about single and double singularities

% Output:
% Singles => [joint # , angle of singularity]
% Doubles => [1st joint, 1st angle, 2nd joint, 2nd angle] 
% Triples => [1st joint, 1st angle, 2nd joint, 2nd angle, 3rd joint, 3rd angle]

% Singularities are found by trying a large number of joint configurations
% and testing to see if each one is a singularity. Specifically, this
% functions holds all the joint angles constant and then varies one or two
% joints at a time in order to find singularities

% WARNING: this function will not find all robot singularities. More may
% exist!!!

% Output:
% if only one value, will return single singularities
% if two values will return single and double singularities
    
    if ~exist('div','var')
        % div parameter does not exist, so default it to something
        div = 96;
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
                i_list = qLister(robot.links(i),ceil(div/16));
                for ii=i_list
                    if ~ismember([i ii],s,'rows')
                        j_list = qLister(robot.links(j),ceil(div/16));
                        for jj=j_list
                            if ~ismember([j jj],s,'rows') && ~ismember([i ii j jj],d,'rows')
                                k_list = qLister(robot.links(k),ceil(div/16));
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

