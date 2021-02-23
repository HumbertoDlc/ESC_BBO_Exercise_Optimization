% Code prepared for Chapter VI of dissertation titled 
% "Simulation and Control at the Boundaries Between Humans and Assistive Robots" by
% Holly Warner
% Cleveland State University
% December 2019
% 
% File description: Returns tendon force (quadratic model)

function out = SEE(LSEE,s,k)

out = k.*(LSEE-s).^2;



