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
%%%%% SV_T:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% SV_I:
for i=1:1:50
    
        try load(['ESC_Optimization_Model_',num2str(i),'_SV_T.mat'])
        catch disp(['Model N',num2str(i),' did not converge.']);
            continue
        end

        if length(T) < 32000
            continue
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%% Plots:
        figure('rend','painters','pos',[10 10 1200 600]);
        plot(T,Thetai.*180/pi,T,Theta_arr.*180/pi,'linewidth',2); ylabel('Theta');
        set(gca,'FontSize',16);   

        pause
        clear
        close all
        
end
