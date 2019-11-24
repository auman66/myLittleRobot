
%ink_kin_pso() - Uses Particle Swarm Optimization to find I.K. solutions 
%                based off a cost function
%%
%[sol]  : Viable solutions based off cost, and 
%         are considered unique
%      
% R     : Serial Link Object
% T_des : Desired end-effector homogenous transformation matrix
% q0    : Initial Starting value (not needed?)
% m     : masking array for joints
function [sol] = inv_kin_pso(R, T_des, q0, m)
    %% Probelm Setup
    prob_struct.CostFunction = @CostFunction;
    prob_struct.nVar  = 3;
    prob_struct.VarMin = -10;
    prob_struct.VarMax =  10; 
    prob_struct.Robot = R;
    prob_struct.T_des = T_des;
    prob_struct.mask_vec = m;
    
    %% Param Initialization
    params_struct.MaxIt = 250;
    params_struct.nPop = 200;
    params_struct.w = 1.0;
    params_struct.wdamp = 0.9;
    params_struct.c1 = 1.0;
    params_struct.c2 = 0.5;
    params_struct.ShowIterInfo = false;
    
    %%
    sol_candidates = PSO(prob_struct, params_struct);
    
    q_best = sol_candidates.BestSol.Position %Global Solution

    %All solutions where the cost is below a reasonable threshold
    %This ignores the best solution so we add it back to the top of the
    %solution array below
    q_list = sol_candidates.pop(([sol_candidates.pop.Cost] < 1E-4)); 
    sol = [q_best; cell2mat({q_list.Position}')]; %Convert q_list.position to an array.
                                             %Stack best solution on top
                                             
    %Wraps col corresponding to revolute joints to [-pi, pi]                                         
    revolute_mask = (R.config == 'R'); %Logical array for joints that are
                                       %revolute
    for i = 1:length(revolute_mask)
        if (revolute_mask(i))
            sol(:, i) = wrapToPi(sol(:, i));
        end
    end
    
    %Remove duplicate entries
    sol = uniquetol(sol, 'ByRows', true);
    
    
end

