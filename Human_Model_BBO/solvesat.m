% Code prepared for Chapter VI of dissertation titled 
% "Simulation and Control at the Boundaries Between Humans and Assistive Robots" by
% Holly Warner
% Cleveland State University
% December 2019
% 
% Simulation code originally from paper "Motion Optimization for Musculoskeletal
% Dynamics: A Flatness-Based Sum-of-Squares Approach"
% by Hanz Richter and Holly Warner
% Cleveland State University, Center for Human-Machine Systems, 2019
% 
% File description: Attempts to find a solution for the neural input that 
% corresponds to a given activation and rate of activation using a saturation
% function for the time constant variation.

function [nsol,solfound] = solvesat(tau_act,beta,a,adot)

% deactivation time constant
tau_deact = tau_act/beta;

% initialize
solfound = 0;
nsol = [];

% find discriminant of quadratic equation associated with a solution in [0,1]
D = (a+(1-a)*beta)^2+4*(1-beta)*adot*tau_act;

if D >=  0 % real solutions exist using the linear function (domain unrestricted)
    nsol1 = (-(beta-(1-beta)*a)+sqrt(D))/2/(1-beta);
    nsol2 = (-(beta-(1-beta)*a)-sqrt(D))/2/(1-beta);
    
    % check range
    if nsol1 <= 1 && nsol1 >= 0 % solution is correct
        nsol = nsol1;
        solfound = 1;
    elseif nsol2 <= 1 && nsol2 >= 0 % solution is correct 
        nsol = nsol2;
        solfound = 1;
    end % note: only one solution may exist
end

if ~solfound % solutions may be found in saturation
    nsol3 = tau_act*adot+a;
    nsol4 = tau_deact*adot+a;
    if nsol3 > 1 % solution is correct
        nsol = nsol3;
        solfound = 1;
    elseif nsol4 < 0 % solution is correct
        nsol  =  nsol4;
        solfound  =  1;
    end
end