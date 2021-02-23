% Code prepared for Chapter VI of dissertation titled 
% "Simulation and Control at the Boundaries Between Humans and Assistive Robots" by
% Holly Warner
% Cleveland State University
% December 2019
% 
% File description: Inverse of tendon force function (outside slack)

function LSEE = SEEinv(FT,s,k)

LSEE = sqrt(FT./k)+s;