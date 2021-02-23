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
%%%%% MV:
for i=1:1:50
    
        try load(['ESC_Optimization_Model_',num2str(i),'_MV.mat'])
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

        figure('rend','painters','pos',[10 10 1200 600]);
        plot(T,Ki,T,K_arr,'linewidth',2); ylabel('K');
        set(gca,'FontSize',16);
        
        pause
        clear
        close all
        
end