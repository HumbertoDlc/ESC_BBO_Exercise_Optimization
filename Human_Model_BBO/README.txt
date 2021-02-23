Code prepared for Chapter VI of dissertation titled 
"Simulation and Control at the Boundaries Between Humans and Assistive Robots" by
Holly Warner
Cleveland State University
December 2019

Run main.m. Options can be set in this file.

Optimization results from dissertation provided in data (.mat) files listed below.

Code tested in MATLAB 2018b.

General sum-of-squares polynomial solution requires installation of the Sum
of Squares Toolbox SOSTools. Code was written using Version 3.03.

SOSTools calls the SeDuMi solver, which must also be installed. Code was written using 
Version 1.32.

Citation
Holly Warner. "Simulation and Control at the Boundaries Between Humans 
and Assistive Robots." Cleveland State University, 2019. 

Code can be run three ways:
1. Validation using trajectories from Levin 2001
    Example: select useSOSTools = 0
                    useTrajOpt = 0
                    levinTraj = 1
                    runMonteCarlo = 0
             run main.m

2. Optimal control solution and simulation of a trajectory found from a previous optimization trial
    Example: select useSOSTools = 0
                    useTrajOpt = 1
                    runMonteCarlo = 0
             uncomment and modify specified lines for loading data in main.m
             run main.m

3. Monte Carlo optimization trial
    Example: select useSOSTools = 0
                    useTrajOpt = 1
                    runMonteCarlo = 1
             assign trialSpecifier string
             modify lines 41 and 97 in optExercise.m as needed
             modify lines 47 and 51 in BBO.m as needed
             run main.m

Files:
    Setup:
        main.m
    Optimal Control Solver:
        calcTauBounds2linkEnum.m
        SOS_Flatness_OL.m
        SOSflatOPT.m
    Trajectory Generation:
        impedanceCtrlTaskSpace
        inverseKinEllipse.m
        inverseKinSymbolics.m
        JvEval.m
        JvSym.m
        trajMain.m
    Trajectory Optimization:
        BBO.m
        ClearDups.m
        ComputeCostAndConstrViol.m
        Conclude.m
        Init.m
        monteCarlo.m
        optExercise.m
        PopSort.m
    System Dynamics:
        mdlStateDer.m
    Muscle Dynamics:
        actdynsat.m
        f1.m
        g1.m
        ginv.m
        PE.m
        SEE.m
        SEEinv.m
        SEEPrime.m
        solvesat.m
    Utilities:
        animate.m
        SetPlotOptions
    Results/ - contains results from reported Monte Carlo optimization trials,
               results can be reviewed by following the directions for the second example above