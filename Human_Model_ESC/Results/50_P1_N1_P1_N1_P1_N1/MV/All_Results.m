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
%%%%% Load data:
for i=1:1:50
        try 
            load(['ESC_Optimization_Model_',num2str(i),'_MV.mat']);
        catch
            continue
        end
        if DATA(1) < 0
            DATA_T(i) = DATA(1) + 180;
            DATA_I(i) = DATA(2);
        else
            DATA_T(i) = DATA(1);
            DATA_I(i) = DATA(2);
        end
end
save('ALL_50_MV_DATA_PN.mat','DATA_T','DATA_I');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% Axis limit:
DATA_X = DATA_T; DATA_X(DATA_X == 0)=inf;
DATA_Y = DATA_I; DATA_Y(DATA_Y == 0)=inf;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% Plots:
size_NZ = 25;
figure('rend','painters','pos',[10 10 1200 600]);
for j=1:1:25
    
    for i=1:size_NZ
        scatter(DATA_T(i),...
                DATA_I(i),150,'filled','MarkerFaceColor',[i,0,0]./size_NZ);
        hold on;
    end
    xlabel('Ellipsoidal Orientation (DEG)'); ylabel('Impedance (Stiffness N/m)');
    set(gca,'FontSize',16);
    
end
figure('rend','painters','pos',[10 10 1200 600]);
for j=26:1:50
    
    for i=1:size_NZ
        scatter(DATA_T(i),...
                DATA_I(i),150,'filled','MarkerFaceColor',[i,0,0]./size_NZ);
        hold on;
    end
    xlabel('Ellipsoidal Orientation (DEG)'); ylabel('Impedance (Stiffness N/m)');
    set(gca,'FontSize',16);
    
end
