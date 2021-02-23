% Code prepared for Chapter VI of dissertation titled 
% "Simulation and Control at the Boundaries Between Humans and Assistive Robots" by
% Holly Warner
% Cleveland State University
% December 2019
% 
% Structure based on example code from the textbook Evolutionary Optimization 
% Algorithms by Dan Simon
% 
% File description: Functions for the definition, initialization, and cost
% function for the optimal exercise problem

function [InitFunction, CostFunction, FeasibleFunction] = optExercise
% assign function handles
InitFunction = @optExerciseInit;
CostFunction = @optExerciseCost;
FeasibleFunction = [];
return

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [Population, OPTIONS] = optExerciseInit(OPTIONS)
% initialize population
OPTIONS.numVar = 18; % problem dimension
% boundaries for each variable
%                       1 aMa   2 bMa   3 xcMa  4 ycMa  5 phiMa 6 aMd   7 bMd   8 xcMd  9 ycMd  10 phiMd    11 f    12 dir  13 Ix   14 Iy   15 Bx   16 By   17 Kx   18 Ky 

%%%%% ZERO:
%OPTIONS.MinDomain = [   5e-3    5e-3    -0.6    0       0       5e-3    5e-3    -0.6    0       0           0.1     -1      0       0       0       0       0       0   ];
%OPTIONS.MaxDomain = [   0.3     0.3     0.6     0.6     pi      0.3     0.3     0.6     0.6     pi          3       1       200     200     500     500     1000      1000];

%%%%% FIXED:
%OPTIONS.MinDomain = [   0.25    0.15    0       0.3      0       0.12    0.12    0      0.3     0           1      1      5       5       1       1       50       50   ];
%OPTIONS.MaxDomain = [   0.25    0.15    0       0.3     2*pi     0.12    0.12    0      0.3     0           1      1      5       5       1       1       50       50];

%%%%% IMP ZERO:
OPTIONS.MinDomain = [   0.25    0.15    0       0.3      0       0.12    0.12    0      0.3     0            0.1    1      100        100        1       1       50       50   ];
OPTIONS.MaxDomain = [   0.25    0.15    0       0.3     2*pi     0.12    0.12    0      0.3     0            3      1      100        100        1       1       50       50];




Population(OPTIONS.popsize).chrom = zeros(1, OPTIONS.numVar);
for popindex = 1 : OPTIONS.popsize
	chrom = OPTIONS.MinDomain + (OPTIONS.MaxDomain - OPTIONS.MinDomain) .* rand(1, OPTIONS.numVar);
    if chrom(12) < 0
        chrom(12) = -1;
    else
        chrom(12) = 1;
    end
    Population(popindex).chrom = chrom;
end
OPTIONS.OrderDependent = true;

% select weights for individual muscles here
OPTIONS.weights = [-1 1 -1 1 -1 1];
return

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [Population,OPTIONS] = optExerciseCost(Population,OPTIONS)
% compute the cost of each member in Population
popsize = length(Population);
for popindex = 1 : popsize
    
    % try to solve open-loop control problem
    try
        % turn off options, for speed
        forwardSim = 0;
        displayPlots = 0;
        showAnimations = 0;
        SOS_Flatness_OL
    catch e
        if strfind(e.message,'Solver failed:')
            constrt = inf; % assign an infinite valued constraint violation if the open-loop solver fails
        elseif ~strfind(e.message,'Trajectory not achievable')
            e % if any other error occurs, output it
        else
            % otherwise, error is associated with an unreachable trajectory
            % evaluate constraint violation associated with the trajectory
            constrt = 10*sum([~isreal(q1Md), ~isreal(q2Md), ~isreal(q1Ma), ~isreal(q2Ma), ...
            ~isreal(q1EllA), ~isreal(q2EllA), sum(q1EllA(1,:) < -30*pi/180) ~= 0, ...
            sum(q2EllA(1,:) < 5*pi/180) ~= 0, sum(diff(q1EllA(1,:)) > 5) ~= 0, ...
            sum(diff(q2EllA(1,:)) > 5) ~= 0]);
        end
        % if there is an error, the cost and constraints are all assigned
        % the constraint violation value
        Population(popindex).cost = constrt;
        Population(popindex).g(1:5) = constrt;
        Population(popindex).g(Population(popindex).g <= 0) = 0;
        % sum of constraints: bigger the sum, bigger the constraint violation
        Population(popindex).G = sum(Population(popindex).g);
        continue
    end
    
    % if there were no errors, compute cost based on muscle activations
    Population(popindex).cost = -1/(6*2*pi/omega)*OPTIONS.weights*trapz(delta,act')';
    
    % compute the level of constraint violation
    % muscle activations and neural inputs between zero and one:
    %g1 <= 0
    Population(popindex).g(1) = -min(min(act));
    %g2 <= 0
    Population(popindex).g(2) = max(max(act)) - 1;
    %g3 <= 0
    Population(popindex).g(3) = -min(min(n));
    %g4 <= 0
    Population(popindex).g(4) = max(max(n)) - 1;
    
    % magnitude of endpoint force <= 45 N:
    %g5 <= 0
    Population(popindex).g(5) = max(sqrt(Fext(1,:).^2+Fext(2,:).^2)) - 45;
    
    Population(popindex).g(Population(popindex).g <= 0) = 0;
    %Sum of constraints: Bigger the sum, bigger the constraint violation
    Population(popindex).G = sum(Population(popindex).g);
end
return