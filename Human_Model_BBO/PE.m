% Code prepared for Chapter VI of dissertation titled 
% "Simulation and Control at the Boundaries Between Humans and Assistive Robots" by
% Holly Warner
% Cleveland State University
% December 2019
% 
% File description: Returns the passive muscle force (parallel stiffness),
% currently assigned zeros due to range of motion of arm


function out = PE(LC,Lo)

out = zeros(length(LC),1);
