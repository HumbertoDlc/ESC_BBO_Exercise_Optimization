% Code prepared for Chapter VI of dissertation titled 
% "Simulation and Control at the Boundaries Between Humans and Assistive Robots" by
% Holly Warner
% Cleveland State University
% December 2019
% 
% Based on example code from the textbook Evolutionary Optimization 
% Algorithms by Dan Simon
% 
% File description: Returns tendon force derivative (quadratic model)

function out = SEEPrime(LSEE,s,k)

out = 2*k.*(LSEE-s);



