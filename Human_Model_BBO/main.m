% Code prepared for Chapter VI of dissertation titled 
% "Simulation and Control at the Boundaries Between Humans and Assistive Robots" by
% Holly Warner
% Cleveland State University
% December 2019
% 
% File description: Main file for setting parameters and specifying the
% subset of code to be run

% clear workspace and command window, close all plots
clear;
clc;
close all;

for i=2:1:10
% define global settings
global useSOSTools useTrajOpt Person_Model

% select whether to use a linear programming solution or SOSTools
% 0: linear programming************
% 1: SOSTools
useSOSTools = 0;

% select whether to use an optimal exercise trajectory found by BBO or not
% 0: Use hard-coded cases from Levin 2001*************
% 1: Use a trajectory found by BBO --> select this option for running a new optimization as well
useTrajOpt = 1;

% if not using an optimal exercise trajectory found by BBO, can select a pre-defined solution from Levin 2001
% 0: *************************
% 1: Vertical collapsed ellipse
% 2: Right-slant collapsed ellipse
% 3: Horizontal collapsed ellipse
% 4: Left-slant collapsed ellipse
levinTraj = 0;

% select whether or not to run Monte Carlo optimization trials
% the number of trials can be specified in monteCarlo.m on line 16
% 0: use a pre-defined solution 
% 1: run monteCarlo.m*******************
runMonteCarlo = 1;
% assign a string to be appended to filename generated after monteCarlo.m completes
trialSpecifier = 'AntDelt';
    
% if running optimization, set weighting factors for individual muscles on
% line 41 and set force magnitude limit on line 97 of optExercise.m.
% 
% population size and number of generations can be specified on lines 47
% and 51, respectively, in BBO.m

% if not running optimization or a trial from Levin 2001, specify
% data to load here, and select the first (best) solution in the population
% load('11-20-2019_12-19_MuscWeights1 -1 -1 -1 -1 -1')
% clear levinTraj
% popindex = 1;



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% Constraints in: optExercise.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% Plots in monteCarlo and trajMain.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% Iteractions: in BBO -> Maxgen = 300;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% execute code under chosen settings
    if runMonteCarlo
        clear levinTraj
        Person_Model = i;
        monteCarlo
    else
        clear trialSpecifier
        displayPlots = 0;
        showAnimations = 0;
        forwardSim = 0;
        Person_Model = i;
        SOS_Flatness_OL
    end
end