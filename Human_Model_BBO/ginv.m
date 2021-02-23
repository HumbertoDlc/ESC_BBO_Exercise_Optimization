% Code prepared for Chapter VI of dissertation titled 
% "Simulation and Control at the Boundaries Between Humans and Assistive Robots" by
% Holly Warner
% Cleveland State University
% December 2019
% 
% File description: Inverse of velocity dependence function

function out = ginv(z,Vm)

A = 0.25; % Hill constant
gmax = 1.5; % Maximum eccentric to isometric force ratio

for idx = 1:length(z)
    if z(idx)<=1
        % CE shortens (Hill model)
        out(idx) = (Vm(idx)-z(idx).*Vm(idx))./(1+z(idx)/A); 
    else
        % CE lengthens (Katz model)
        out(idx) = -(z(idx)-1)*(gmax-1)*A.*Vm(idx)./((A+1)*(gmax-z(idx))); 
    end
end

out = out';
