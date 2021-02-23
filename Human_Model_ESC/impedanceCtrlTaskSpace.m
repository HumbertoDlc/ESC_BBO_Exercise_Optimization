function Fext = impedanceCtrlTaskSpace(xR,yR,xM,yM,I,B,K)

xtilde = [xR(1,:)-xM(1,:); yR(1,:)-yM(1,:)];
xtildeD = [xR(2,:)-xM(2,:);yR(2,:)-yM(2,:)];
xtildeDD = [xR(3,:)-xM(3,:);yR(3,:)-yM(3,:)];

Fext = -(I*xtildeDD + B*xtildeD + K*xtilde);

end