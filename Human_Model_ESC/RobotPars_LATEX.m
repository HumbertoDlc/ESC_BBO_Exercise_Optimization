%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%                                                                 %%%%%
%%%%%                        HUMAN  CONTROLLER                        %%%%%
%%%%%                                                                 %%%%%
%%%%%                     Humberto J De las Casas                     %%%%%
%%%%%                                                                 %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all
clear
clc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% Load parameters:
load('robotpars.mat');

for j=1:1:25
    
   i = j+25;
   disp([num2str(j),' & ',num2str(robotpars(i).m1),' & ',num2str(robotpars(i).m2)...
        ,' & ',num2str(robotpars(i).I1),' & ',num2str(robotpars(i).I2)...
        ,' & ',num2str(robotpars(i).l1),' & ',num2str(robotpars(i).l2)...
        ,' & ',num2str(robotpars(i).lc1),' & ',num2str(robotpars(i).lc2),' \\ ']);
   

   
end