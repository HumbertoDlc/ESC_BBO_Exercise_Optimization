% Code prepared for Chapter VI of dissertation titled 
% "Simulation and Control at the Boundaries Between Humans and Assistive Robots" by
% Holly Warner
% Cleveland State University
% December 2019
% 
% File description:  Returns the velocity dependence function

function out = g1(u,Vm)  

% u is the contraction velocity = -LCdot

A = 0.25;	% Hill constant
gmax = 1.5; % Maximum eccentric to isometric force ratio

for idx = 1:length(u)
    if u(idx) < 0
        out(idx) = (A*Vm(idx) - A*Vm(idx)*gmax + u(idx)*gmax + A*u(idx)*gmax)./(A*Vm(idx) + u(idx) + A*u(idx) - A*Vm(idx)*gmax);     %CE lengthens (Katz model)
    else 
        out(idx) = (A*Vm(idx) - A*u(idx))./(A*Vm(idx) + u(idx));       % CE shortens (Hill model)
    end
end

out = out';    