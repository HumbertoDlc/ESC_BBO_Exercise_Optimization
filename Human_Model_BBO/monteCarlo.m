% Code prepared for Chapter VI of dissertation titled 
% "Simulation and Control at the Boundaries Between Humans and Assistive Robots" by
% Holly Warner
% Cleveland State University
% December 2019
%
% File description: Perform Monte-Carlo trajectory optimization trials 

% initialize logging variables
bestSolns = [];
bestCosts = [];
times = [];
constrtViols = [];
successfulTrials = 0;
trialIdx = 0;

while successfulTrials < 5 % change this value to choose the number of solutions to be found
    
    % trial counter
    trialIdx = trialIdx + 1;
    
    % run the optimization
    tic;
    Population = BBO(@optExercise);
    
    % store summary results
    times = [times;toc];
    bestSolns = [bestSolns;Population(1).chrom];
    bestCosts = [bestCosts;Population(1).cost];
    constrtViols = [constrtViols; Population(1).G];
    
    % if BBO finds a feasible solution, perform all checks and show all plots
    if constrtViols(trialIdx) == 0
        forwardSim = 0;
        displayPlots = 0;
        showAnimations = 0;
        popindex = 1;
        try
            SOS_Flatness_OL
        catch e
            e
        end
        successfulTrials = successfulTrials + 1;
    end
    
    % clean up for next trial
    clearvars -except times bestSolns bestCosts constrtViols trialIdx successfulTrials trialSpecifier useSOSTools
    clear PopSort
end

% save summary of Monte Carlo trial
save([datestr(now,'mm-dd-yyyy_HH-MM') '_MonteCarlo' trialSpecifier])