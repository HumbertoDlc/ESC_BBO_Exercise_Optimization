% Code prepared for Chapter VI of dissertation titled 
% "Simulation and Control at the Boundaries Between Humans and Assistive Robots" by
% Holly Warner
% Cleveland State University
% December 2019
% 
% Simulation code originally from paper "Motion Optimization for Musculoskeletal
% Dynamics: A Flatness-Based Sum-of-Squares Approach"
% by Hanz Richter and Holly Warner
% Cleveland State University, Center for Human-Machine Systems, 2019
% 
% File description: State derivatives for the 2-link, 6-muscle planar arm

function xdot = mdlStateDer(t,x,externalTorque,n_time,n,robotpars,musclepars,g,tau_act,beta)

% linkage parameters
m1 = robotpars.m1;
m2 = robotpars.m2;
l1 = robotpars.l1;
lc1 = robotpars.lc1;
l2 = robotpars.l2;
lc2 = robotpars.lc2;
I1 = robotpars.I1;
I2 = robotpars.I2;

% muscle parameters
d1 = musclepars.d1;
d2 = musclepars.d2;
a0 = musclepars.a0;
Ls = musclepars.Ls;
Lo = musclepars.Lo;
Vm = musclepars.Vm;
W = musclepars.W;
Fmax = musclepars.Fmax;
k = musclepars.k;

% parse states
z = x(1:4);
z_1 = z(1:2);
z_2 = z(3:4);
LS = x(5:10);
a = x(11:end);

% find numerical values for matrix C and vector g
h = -m2*l1*lc2*sin(z_1(2));
C(1,1) = h*z_2(2);
C(1,2) = h*z_2(2)+h*z_2(1);
C(2,1) = -h*z_2(1);
C(2,2) = 0;

gg(1,1) = (m1*lc1+m2*l1)*g*cos(z_1(1))+m2*lc2*g*cos(z_1(1)+z_1(2));
gg(2,1) = m2*lc2*g*cos(z_1(1)+z_1(2));

% symbolic mass matrix inverse
Dinv = [(m2*lc2^2 + I2)/(I1*I2 + l1^2*lc2^2*m2^2 + I2*l1^2*m2 + I2*lc1^2*m1 + I1*lc2^2*m2 + lc1^2*lc2^2*m1*m2 - l1^2*lc2^2*m2^2*cos(z_1(2))^2),                            -(m2*lc2^2 + l1*m2*cos(z_1(2))*lc2 + I2)/(I1*I2 + l1^2*lc2^2*m2^2 + I2*l1^2*m2 + I2*lc1^2*m1 + I1*lc2^2*m2 + lc1^2*lc2^2*m1*m2 - l1^2*lc2^2*m2^2*cos(z_1(2))^2);
-(m2*lc2^2 + l1*m2*cos(z_1(2))*lc2 + I2)/(I1*I2 + l1^2*lc2^2*m2^2 + I2*l1^2*m2 + I2*lc1^2*m1 + I1*lc2^2*m2 + lc1^2*lc2^2*m1*m2 - l1^2*lc2^2*m2^2*cos(z_1(2))^2), (m2*l1^2 + 2*m2*cos(z_1(2))*l1*lc2 + m1*lc1^2 + m2*lc2^2 + I1 + I2)/(I1*I2 + l1^2*lc2^2*m2^2 + I2*l1^2*m2 + I2*lc1^2*m1 + I1*lc2^2*m2 + lc1^2*lc2^2*m1*m2 - l1^2*lc2^2*m2^2*cos(z_1(2))^2)];

% take into account external forces at linkage endpoint
Phi = SEE(LS,Ls,k);
ArmMatrix = [d1';d2'];
tau = ArmMatrix*Phi;
JtFext = interp1(n_time,externalTorque',t)';

% compute the linkage acceleration from Mqddot + Cqdot + g = tau + JTFext
z2dot = Dinv*(-C*z_2 - gg + tau + JtFext);

% contractile element length
LC = a0-d1*z_1(1)-d2*z_1(2)-LS;

% parallel force
FP = PE(LC,Lo);

% compute z
z = (Phi-FP)./(f1(LC,W,Lo).*a.*Fmax);

% compute u
u = ginv(z,Vm);

% compute Lsdot
Lsdot = -d1*z_2(1)-d2*z_2(2)+u; % recall u = -Lcdot

% interpolate to find proper n inputs, then find adot
for i = 1:6
    n_input = interp1(n_time,n(:,i),t);    
    adot(i) = actdynsat(n_input,a(i),tau_act,beta);
end

% assemble derivatives
xdot = [z_2;z2dot;Lsdot;adot'];
