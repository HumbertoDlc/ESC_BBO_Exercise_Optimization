%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%                                                                 %%%%%
%%%%%                         ESC BBO RESULTS                         %%%%%
%%%%%                                                                 %%%%%
%%%%%                     Humberto J De las Casas                     %%%%%
%%%%%                                                                 %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all
clear
clc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% Load Data:
load('DATA_ESC_BBO');
ESC(1) = 0;
BBO(1) = 0;
Mean_error = mean(abs(ESC-BBO));
Variance_error = var(abs(ESC-BBO));
STD_error = std(abs(ESC-BBO));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% Plot:
c = categorical(["1","2","3","4","5"]);
c = reordercats(c,{'1','2','3','4','5'});
figure('Renderer', 'painters', 'Position', [50 50 1500 600]);
b=bar(c,[BBO(2),ESC(2);BBO(3),ESC(3);...
         BBO(4),ESC(4);BBO(6),ESC(6);BBO(7),ESC(7)]);
ylabel('Theta (DEG)');title('Optimal Ellipsoidal Orientation');
xlabel('Arm Model Number')
legend('BBO Framework', 'ESC Framework','location','southeast')
set(gca,'FontSize',30);
