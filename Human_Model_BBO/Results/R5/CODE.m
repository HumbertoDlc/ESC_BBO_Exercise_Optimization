close all
clear
clc

warning('off')
load('Results_1');
disp(['Frequency = ',num2str(Chrom(11)),'. Solution = ',num2str(Chrom(5)*180/pi),'. Cost = ',num2str(min(MinCost))])
disp(['Frequency = ',num2str(Chrom2(11)),'. Solution = ',num2str(Chrom2(5)*180/pi),'. Cost = ',num2str(min(MinCost))])
disp(' ')

load('Results_2');

disp(['Frequency = ',num2str(Chrom(11)),'. Solution = ',num2str(Chrom(5)*180/pi),'. Cost = ',num2str(min(MinCost))])
disp(['Frequency = ',num2str(Chrom2(11)),'. Solution = ',num2str(Chrom2(5)*180/pi),'. Cost = ',num2str(min(MinCost))])
disp(' ')

load('Results_3');

disp(['Frequency = ',num2str(Chrom(11)),'. Solution = ',num2str(Chrom(5)*180/pi),'. Cost = ',num2str(min(MinCost))])
disp(['Frequency = ',num2str(Chrom2(11)),'. Solution = ',num2str(Chrom2(5)*180/pi),'. Cost = ',num2str(min(MinCost))])
disp(' ')

load('Results_4');

disp(['Frequency = ',num2str(Chrom(11)),'. Solution = ',num2str(Chrom(5)*180/pi),'. Cost = ',num2str(min(MinCost))])
disp(['Frequency = ',num2str(Chrom2(11)),'. Solution = ',num2str(Chrom2(5)*180/pi),'. Cost = ',num2str(min(MinCost))])
disp(' ')

load('Results_5');

disp(['Frequency = ',num2str(Chrom(11)),'. Solution = ',num2str(Chrom(5)*180/pi),'. Cost = ',num2str(min(MinCost))])
disp(['Frequency = ',num2str(Chrom2(11)),'. Solution = ',num2str(Chrom2(5)*180/pi),'. Cost = ',num2str(min(MinCost))])
disp(' ')

load('Results_6');

disp(['Frequency = ',num2str(Chrom(11)),'. Solution = ',num2str(Chrom(5)*180/pi),'. Cost = ',num2str(min(MinCost))])
disp(['Frequency = ',num2str(Chrom2(11)),'. Solution = ',num2str(Chrom2(5)*180/pi),'. Cost = ',num2str(min(MinCost))])
disp(' ')

load('ESC_Optimization_Model_5_SV_T');
disp(['Solution = ',num2str(DATA(1)),'. Cost = ',num2str(min(M_Per))])
