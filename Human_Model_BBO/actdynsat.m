% Code used for Chapter VI of dissertation titled 
% "Simulation and Control at the Boundaries Between Humans and Assistive Robots" by
% Holly Warner
% Cleveland State University
% December 2019
% 
% File description: Saturation function to represent the change in time
% constant in the activation dynamics

function adot = actdynsat(n,a,tau_act,beta)

% deactivation time constant
tau_deact = tau_act/beta;

% linear segment with constant values at each end
if n < 0
    sigm = 1/tau_deact;
elseif n >= 0 && n <= 1
    sigm = (beta+(1-beta)*n)/tau_act;
else
    sigm = 1/tau_act;
end

adot = sigm*(n-a);

    