% Code prepared for Chapter VI of dissertation titled 
% "Simulation and Control at the Boundaries Between Humans and Assistive Robots" by
% Holly Warner
% Cleveland State University
% December 2019
% 
% File description: Impedance control in task space (Cartesian)

function Fext = impedanceCtrlTaskSpace(xElla,yElla,xElld,yElld,I,B,K)

% position, velocity, and acceleration difference between actual and
% desired ellipses
xtilde = [xElla(1,:)-xElld(1,:); yElla(1,:)-yElld(1,:)];
xtildeD = [xElla(2,:)-xElld(2,:);yElla(2,:)-yElld(2,:)];
xtildeDD = [xElla(3,:)-xElld(3,:);yElla(3,:)-yElld(3,:)];

% force applied to arm endpoint due to impedance control
for idx = 1:length(xtilde)
    Fext(:,idx) = -(I*xtildeDD(:,idx) + B*xtildeD(:,idx) + K*xtilde(:,idx));
end

end