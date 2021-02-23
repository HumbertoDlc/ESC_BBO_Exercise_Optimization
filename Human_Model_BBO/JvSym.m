% Code prepared for Chapter VI of dissertation titled 
% "Simulation and Control at the Boundaries Between Humans and Assistive Robots" by
% Holly Warner
% Cleveland State University
% December 2019
% 
% File description: Run to symbolically generate a function called JvEval to
% compute the velocity Jacobian for a 2 DOF planar linkage

% prepare workspace and command window
clear;
clc;

% define symbols
syms q1 q2 l1 l2
pi = sym('pi');

% homogeneous transformations
H10=HRz(q1)*HTx(l1);
H21=HRz(q2)*HTx(l2);
H20=H10*H21;

% frame origins in world coordinates
o00 = [0;0;0];
o10 = H10(1:3,4);
o20 = H20(1:3,4);

% CM origins in world coordinates
c10=H10*[-l1/2;0;0;1];
c10=c10(1:3);
c20=H20*[-l2/2;0;0;1];
c20=c20(1:3);

% actuation axes
z0=[0;0;1];
z1=H10(1:3,1:3)*[0;0;1];

% Jacobians of endpoint
c20End=H20*[0;0;0;1];
c20End=c20End(1:3);

Jv1=cross(z0,c20End-o00);
Jv2=cross(z1,c20End-o10); 

Jv = [Jv1 Jv2];

% automatically code function for evaluating velocity Jacobian
JvEval = matlabFunction(subs(Jv(1:2,1:2)),'File','JvEval','Vars',{[q1;q2],l1,l2});