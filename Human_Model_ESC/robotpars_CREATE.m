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
%%%%% Load parameters:
load('Link_Lengths.mat');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% Base model:
base.m1 = 2.24;
base.m2 = 1.76;
base.l1 = 0.33;
base.lc1 = 0.1439;
base.l2 = 0.32;
base.lc2 = 0.2182;
base.I1 = 0.0253;
base.I2 = 0.0395;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% Male subject parameters:
for i=1:1:length(Male_L1)

    scale_l1 = Male_L1(i)/base.l1;
    scale_l2 = Male_L2(i)/base.l2;
    %%%%% +- 5%.
    robotpars(i).m1  = base.m1*scale_l1 + (rand*base.m1-base.m1/2)/10;
    robotpars(i).m2  = base.m2*scale_l2 + (rand*base.m2-base.m2/2)/10;
    robotpars(i).l1  = Male_L1(i);
    robotpars(i).lc1 = base.lc1*scale_l1 + (rand*base.lc1-base.lc1/2)/10;
    robotpars(i).l2  = Male_L2(i);
    robotpars(i).lc2 = base.lc2*scale_l2 + (rand*base.lc2-base.lc2/2)/10;
    robotpars(i).I1  = base.I1*scale_l1 + (rand*base.I1-base.I1/2)/10;
    robotpars(i).I2  = base.I2*scale_l2 + (rand*base.I2-base.I2/2)/10;

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% Female subject parameters:
for i=1:1:length(Female_L1)

    scale_l1 = Female_L1(i)/base.l1;
    scale_l2 = Female_L2(i)/base.l2;
    %%%%% +- 5%.
    robotpars(i+length(Male_L1)).m1  = base.m1*scale_l1 + (rand*base.m1-base.m1/2)/10;
    robotpars(i+length(Male_L1)).m2  = base.m2*scale_l2 + (rand*base.m2-base.m2/2)/10;
    robotpars(i+length(Male_L1)).l1  = Female_L1(i);
    robotpars(i+length(Male_L1)).lc1 = base.lc1*scale_l1 + (rand*base.lc1-base.lc1/2)/10;
    robotpars(i+length(Male_L1)).l2  = Female_L2(i);
    robotpars(i+length(Male_L1)).lc2 = base.lc2*scale_l2 + (rand*base.lc2-base.lc2/2)/10;
    robotpars(i+length(Male_L1)).I1  = base.I1*scale_l1 + (rand*base.I1-base.I1/2)/10;
    robotpars(i+length(Male_L1)).I2  = base.I2*scale_l2 + (rand*base.I2-base.I2/2)/10;

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% Save parameters:
save('robotpars.mat','robotpars');