function [out] = PSO_Mat(problem, params)

    %% Problem Definiton

    CostFunction = problem.CostFunction;  % Cost Function

    nVar = problem.nVar;        % Number of Unknown (Decision) Variables

    VarSize = [1 nVar];         % Matrix Size of Decision Variables

    VarMin = problem.VarMin;	% Lower Bound of Decision Variables
    VarMax = problem.VarMax;    % Upper Bound of Decision Variables


    %% Parameters of PSO

    MaxIt = params.MaxIt;   % Maximum Number of Iterations

    nPop = params.nPop;     % Population Size (Swarm Size)

    w = params.w;           % Intertia Coefficient
    wdamp = params.wdamp;   % Damping Ratio of Inertia Coefficient
    c1 = params.c1;         % Personal Acceleration Coefficient
    c2 = params.c2;         % Social Acceleration Coefficient

    % The Flag for Showing Iteration Information
    ShowIterInfo = params.ShowIterInfo;    

    MaxVelocity = 0.2*(VarMax-VarMin);
    MinVelocity = -MaxVelocity;
    
    %% Initialization

    % The Particles
    particles.Positions = [];
    particles.Velocities = [];
    particles.Costs = [];
    particles.Best_Positions = [];
    particles.Best_Costs = [];

    % Initialize Global Best
    GlobalBest.Cost = inf;

    %% Initialize Population Members


    % Generate Random Solution
    particles.Positions = unifrnd(VarMin, VarMax, [nPop, nVar]);

    % Initialize Velocity
    particles.Velocities = zeros([nPop, nVar]);
    
    for i=1:nPop
        % Evaluation
        particles.Costs(i,1) = CostFunction(problem.Robot, particles.Positions(i, 1:nVar), problem.mask_vec, problem.T_des);
    end
    
    % Update the Personal Best
    particles.Best_Positions = particles.Positions;
    particles.Best_Costs = particles.Costs;

%         % Update Global Best
    if particles.Best_Costs(i) < GlobalBest.Cost
        GlobalBest.Cost = particles.Best_Costs(i);
    end


    % Array to Hold Best Cost Value on Each Iteration
    %BestCosts = zeros(MaxIt, 1);
    

    %% Main Loop of PSO
Total_Pos = [];
    for it=1:MaxIt
        Total_Pos = [Total_Pos, particles.Positions];
        
        %First find the dist bewtween a particle and all other particles...
        dist_mat = pdist2(particles.Positions, particles.Positions);
        
        %Get the ones close enough together
        valid_dist = dist_mat.*(dist_mat<0.1);
      
        local_cost = repmat([particles.Costs], 1, nPop) .* (valid_dist > 0);
        
        [m,best_local_particles_i] = min(local_cost);
        
        best_local_particles_pos = particles.Positions(best_local_particles_i, 1:nVar);

        particles.Velocities =   w*particles.Velocities  ...
              + c1*rand(nPop, 3).*(particles.Best_Positions - particles.Positions) ...
              + c2*rand(nPop, 3).*(best_local_particles_pos - particles.Positions);
        %Min/Max check on velocities...
        particles.Velocities(particles.Velocities > MaxVelocity) = MaxVelocity;
        particles.Velocities(particles.Velocities < MinVelocity) = MinVelocity;
        
        particles.Positions = particles.Positions + particles.Velocities;
        %Wraps Revolute Angles to [-pi, pi]
        revolute_mask = (problem.Robot.config == 'R'); %Logical array for joints that are
                                                       %revolute
        for i = 1:length(revolute_mask)
            if (revolute_mask(i))
                particles.Positions(:, i) = wrapToPi(particles.Positions(:, i));
            end
        end


        for i=1:nPop

            % Evaluation
            particles.Costs(i) = CostFunction(problem.Robot, particles.Positions(i, 1:nVar), problem.mask_vec, problem.T_des);

            % Update Personal Best
            if particles.Costs(i) < particles.Best_Costs(i)

                particles.Best_Positions(i, 1:nVar) = particles.Positions(i, 1:nVar);
                particles.Best_Costs(i) = particles.Costs(i);

                % Update Global Best
                if (particles.Best_Costs(i) < GlobalBest.Cost)
                    GlobalBest.Cost = particles.Best_Costs(i);
                end         

            end

        end

        % Store the Best Cost Value
        BestCosts(it) = min(particles.Best_Costs);

        % Display Iteration Information
        if ShowIterInfo
            disp(['Iteration ' num2str(it) ': Best Cost = ' num2str(BestCosts(it))]);
        end

        % Damping Inertia Coefficient
        w = wdamp*w;%exp((1/it)*MaxIt);
        size(particles.Positions(:, 1));
        
        if mod(it-1, 50) == 0
            figure(it);
            scatter3(particles.Positions(:, 1), particles.Positions(:, 2), particles.Positions(:, 3));
            axis([-5, 5, -5, 5, -5, 5]);
            title(it)
        end

    end
    
    out.pop = particles;
    out.BestSol = GlobalBest;
    out.BestCosts = BestCosts;
    out.Total_Pos = Total_Pos;
    
end