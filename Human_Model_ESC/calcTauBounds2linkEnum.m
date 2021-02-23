% Code prepared for Chapter VI of dissertation titled 
% "Simulation and Control at the Boundaries Between Humans and Assistive Robots" by
% Holly Warner
% Cleveland State University
% December 2019
% 
% File description: Finds torque bounds for a two-link planar robot with
% force applied at endpoint


function [tau1,tau2,DfTau] = calcTauBounds2linkEnum(robotpars,q1,q2,g,externalTorque,Df)

tau = robottorque(q1,q2,robotpars,g) - externalTorque;
tauBounds = Df*tau;

DfTau = min(tauBounds,[],2);

tau1 = tau(1,:);
tau2 = tau(2,:);

end

function tau = robottorque(q1,q2,robotpars,g)

% linkage parameters
m1 = robotpars.m1;
m2 = robotpars.m2;
l1 = robotpars.l1;
lc1 = robotpars.lc1;
lc2 = robotpars.lc2;
I1 = robotpars.I1;
I2 = robotpars.I2;

% state vector
z = [q1(1);q2(1);q1(2);q2(2)];

% divide into position and velocity
n = length(z);
z_1 = z(1:n/2);
z_2 = z(n/2+1:n);

% find numerical values for matrices M and C and vector g
D(1,1) = m1*lc1^2+m2*(l1^2+lc2^2+2*l1*lc2*cos(z_1(2)))+I1+I2;
D(1,2) = m2*(lc2^2+l1*lc2*cos(z_1(2)))+I2;
D(2,1) = D(1,2);
D(2,2) = m2*lc2^2+I2;

h = -m2*l1*lc2*sin(z_1(2));
C(1,1) = h*z_2(2);
C(1,2) = h*z_2(2)+h*z_2(1);
C(2,1) = -h*z_2(1);
C(2,2) = 0;

gg(1,1) = (m1*lc1+m2*l1)*g*cos(z_1(1))+m2*lc2*g*cos(z_1(1)+z_1(2));
gg(2,1) = m2*lc2*g*cos(z_1(1)+z_1(2));

% calculate the torque
tau = D*[q1(3);q2(3)]+C*z_2+gg;

end

