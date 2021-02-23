% Code prepared for Chapter VI of dissertation titled 
% "Simulation and Control at the Boundaries Between Humans and Assistive Robots" by
% Holly Warner
% Cleveland State University
% December 2019
% 
% File description: Returns the length dependence factor given the contractile element length LC

function out = f1(LC,W,Lo)

% W is the width parameter
out = exp(-((LC-Lo)./(W.*Lo)).^2);
