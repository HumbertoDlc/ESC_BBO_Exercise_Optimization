%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%                                                                 %%%%%
%%%%%                        HUMAN  CONTROLLER                        %%%%%
%%%%%                                                                 %%%%%
%%%%%                     Humberto J De las Casas                     %%%%%
%%%%%                                                                 %%%%%
%%%%%                                                                 %%%%%
%%%%% Using the human model from:                                     %%%%%
%%%%% Holly Warner. "Simulation and Control at the Boundaries Between %%%%%
%%%%% Humans and Assistive Robots." Cleveland State University, 2019. %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all
clear
clc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% Subject parameters:
load('robotpars.mat')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% Models:
%%%%% Males: 25 firsts.
%%%%% Females: 25 lasts.
for i=1:1:length(robotpars)
    
    try FCN_Human_Control_ESC(robotpars(i),i,1,0)
    catch disp(['Model N',num2str(i),' did not converge (trajectory).']); 
    end
    
    try FCN_Human_Control_ESC(robotpars(i),i,0,1)
    catch disp(['Model N',num2str(i),' did not converge (impedance).']); 
    end
    
    try FCN_Human_Control_ESC(robotpars(i),i,1,1)
    catch disp(['Model N',num2str(i),' did not converge (MV).']); 
    end
    
end
