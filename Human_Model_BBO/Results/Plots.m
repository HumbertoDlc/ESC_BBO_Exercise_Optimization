%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%                                                                 %%%%%
%%%%%                    ANN  Parameter Estimation                    %%%%%
%%%%%                                                                 %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all
clear
clc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% Load data:
load('Final_Results.mat');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% Categorical:
c = categorical(["Model 1","Model 2","Model 3","Model 4","Model 5"]);
c = reordercats(c,{'Model 1','Model 2','Model 3','Model 4','Model 5'});

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% Ellipse dimensions (cm):
Ellx = 52;
Elly = 18;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% Plots:
    figure('Renderer', 'painters', 'Position', [50 50 1600 500]);
    subplot(4,1,1:3)
    bar(c,Data);
    legend('BBO','ESC','location','southeast')
    ylabel('Theta (DEG)');
    title('Optimal Ellipsoidal Orientation');
    %xlabel('Arm Model Number')
    set(gca,'FontSize',16);
    
    subplot(4,1,4)
    for i=1:length(Data(:,1))
        Theta1 = Data(i,1)*pi/180;
        ki = 0;
        clear EllxF1 EllyF1 EllxF2 EllyF2
        for j=0:0.01:2*pi
            ki = ki+1;
            EllxF1(ki) = -Ellx*cos(j)*cos(Theta1)-Elly*sin(j)*sin(Theta1);
            EllyF1(ki) = -Ellx*cos(j)*sin(Theta1)+Elly*sin(j)*cos(Theta1);
        end
        Theta2 = Data(i,2)*pi/180;
        ki = 0;
        for j=0:0.01:2*pi
            ki = ki+1;
            EllxF2(ki) = -Ellx*cos(j)*cos(Theta2)-Elly*sin(j)*sin(Theta2);
            EllyF2(ki) = -Ellx*cos(j)*sin(Theta2)+Elly*sin(j)*cos(Theta2);
        end
    
        ref_dis = 390;
        plot(EllxF1+(i-1)*ref_dis,EllyF1,'color',[0, 0.4470, 0.7410],'linewidth',2); hold on;
        plot(EllxF2+(i-1)*ref_dis,EllyF2,'color',[0.8500, 0.3250, 0.0980],'linewidth',2);
        set(gca,'FontSize',16);
        axis('equal','off','tight');
        axis([-inf inf -60 60]);
    end
    
