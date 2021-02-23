% Code prepared for Chapter VI of dissertation titled 
% "Simulation and Control at the Boundaries Between Humans and Assistive Robots" by
% Holly Warner
% Cleveland State University
% December 2019
% 
% Based on example code from the textbook Evolutionary Optimization 
% Algorithms by Dan Simon
% 
% File description: Set Matlab plot options according to preferences

function SetPlotOptions
% close all
set( 0, 'DefaultAxesFontSize', 14 )
set( 0, 'DefaultAxesFontName',  'Times New Roman' )
set( 0, 'DefaultAxesFontAngle', 'normal' )
gridState = 'off' ;
set( 0, 'DefaultAxesXGrid', gridState )
set( 0, 'DefaultAxesYGrid', gridState )
set( 0, 'DefaultAxesZGrid', gridState )
set( 0, 'DefaultTextFontSize', 14 )
set( 0, 'DefaultTextFontName', 'Times New Roman' )
set( 0, 'DefaultAxesColor',   'w', ...
        'DefaultAxesXColor',  'k', ...
        'DefaultAxesYColor',  'k', ...
        'DefaultAxesZColor',  'k', ...
        'DefaultTextColor',   'black')
%         'DefaultLineColor',   'black' )
set( 0, 'DefaultLineLineWidth', 1 )
set( 0, 'DefaultUicontrolBackgroundColor', 'w' )
set( 0, 'DefaultAxesBox', 'off') % doesn't work