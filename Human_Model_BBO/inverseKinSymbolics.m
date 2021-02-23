% Code prepared for Chapter VI of dissertation titled 
% "Simulation and Control at the Boundaries Between Humans and Assistive Robots" by
% Holly Warner
% Cleveland State University
% December 2019
% 
% File description: Run to symbolically generate a function called invKinEllipse.m
% for evaluating the inverse kinematics of an ellipse (joint positions, 
% velocities, and accelerations from endpoint trajectory)

% prepare workspace and command window
clear;
clc;

% define symbolic variables
syms x y t a b omega phi xc yc ox oy l1 l2 dir

% determine Cartesian coordinates of ellipse trajectory given major and
% minor axes, center location, phase, and frequency
coords = [cos(phi) -sin(phi); sin(phi) cos(phi)]*[a*cos(dir*omega*t);b*sin(dir*omega*t)] + [xc;yc];

% extract equations
x = coords(1,:);
y = coords(2,:);

% symbolic derivatives
xdot = diff(x,t);
ydot = diff(y,t);

xddot = diff(xdot,t);
yddot = diff(ydot,t);

% shift to origin
xShift = x - ox;
yShift = y - oy;

% inverse kinematics method from Spong, Hutchinson, and Vidyasagar's Robot
% Modeling and Control text, elbow down solution
gamma = atan2(yShift,xShift);
beta = acos((l1^2+l2^2-xShift.^2-yShift.^2)/(2*l1*l2));
alpha = acos((xShift.^2+yShift.^2+l1^2-l2^2)./(2*l1*sqrt(xShift.^2+yShift.^2)));

q1 = gamma - alpha;
q2 = pi - beta;

% symbolic derivatives
q1dot = diff(q1,t);
q2dot = diff(q2,t);

q1ddot = diff(q1dot,t);
q2ddot = diff(q2dot,t);

% collect resulting equations
x = [x;xdot;xddot];
y = [y;ydot;yddot];

q1 = [q1;q1dot;q1ddot];
q2 = [q2;q2dot;q2ddot];

% automatically code a function for x, y, q1, and q2
f = matlabFunction(x,y,q1,q2,'File','inverseKinEllipse','Vars',{t,a,b,dir,omega,phi,xc,yc,ox,oy,l1,l2});